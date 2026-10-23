#!/usr/bin/env python3
"""mavlink_to_tak.py: send a MAVLink vehicle to a TAK Server (or WinTAK) as a Cursor on Target track.

Reads MAVLink from a telemetry radio on a COM port (an RFD900 / SiK radio works), builds a CoT event for the vehicle
every second or so, and sends it with PyTAK to a TAK Server over TLS (client certificate) or to a plain TCP / UDP / multicast
address. Receive-only by default: nothing is transmitted to the vehicle unless MAV_REQUEST_STREAMS is turned on.

    python mavlink_to_tak.py --config config.ini

See README.md for setup. Written for PyTAK 7.x and pymavlink 2.4.x.
"""
import argparse
import asyncio
import configparser
import datetime as dt
import getpass
import logging
import math
import os
import sys
import threading
import time
from xml.sax.saxutils import quoteattr, escape

import pytak
from pymavlink import mavutil

LOG = logging.getLogger("mavlink_to_tak")
SECTION = "mavlink_to_tak"

# MAVLink vehicle types that are not the vehicle itself (GCS, onboard computer, ADS-B, gimbal, camera, ...)
NOT_A_VEHICLE = {6, 18, 26, 27, 28, 30, 31, 32, 33, 34}
M_PER_S_TO_KT = 1.943844


# ------------------------------------------------------------------------------------------------------------------
# MGRS (WGS84 lat/lon to MGRS; the same conversion as the radio and ArduPilot scripts, checked against the "mgrs" library)
# ------------------------------------------------------------------------------------------------------------------
_E2 = 0.0066943799901413
_EP2 = _E2 / (1 - _E2)
_M0 = 1 - _E2 / 4 - 3 * _E2 ** 2 / 64 - 5 * _E2 ** 3 / 256
_M2 = 3 * _E2 / 8 + 3 * _E2 ** 2 / 32 + 45 * _E2 ** 3 / 1024
_M4 = 15 * _E2 ** 2 / 256 + 45 * _E2 ** 3 / 1024
_M6 = 35 * _E2 ** 3 / 3072


def to_mgrs(lat, lon, digits=5):
    """MGRS string such as '18S UJ 23480 06470', or None outside 80 S to 84 N. digits = 1..5 per axis (5 = 1 m)."""
    if lat < -80 or lat >= 84:
        return None
    digits = max(1, min(5, int(digits)))
    zone = int(math.floor((lon + 180) / 6)) % 60 + 1
    if 56 <= lat < 64 and 3 <= lon < 12:
        zone = 32  # Norway
    if lat >= 72 and 0 <= lon < 42:  # Svalbard
        zone = 31 if lon < 9 else 33 if lon < 21 else 35 if lon < 33 else 37
    p = math.radians(lat)
    cp, tp = math.cos(p), math.tan(p)
    nu = 6378137 / math.sqrt(1 - _E2 * math.sin(p) ** 2)
    t, c = tp * tp, _EP2 * cp * cp
    a = math.radians(lon - ((zone - 1) * 6 - 177)) * cp
    a2 = a * a
    m = 6378137 * (_M0 * p - _M2 * math.sin(2 * p) + _M4 * math.sin(4 * p) - _M6 * math.sin(6 * p))
    x = 500000 + 0.9996 * nu * a * (1 + a2 * ((1 - t + c) / 6 + a2 * (5 - 18 * t + t * t + 72 * c - 58 * _EP2) / 120))
    y = 0.9996 * (m + nu * tp * a2 * (0.5 + a2 * ((5 - t + 9 * c + 4 * c * c) / 24 + a2 * (61 - 58 * t + t * t + 600 * c - 330 * _EP2) / 720)))
    if lat < 0:
        y += 10000000
    band = min(19, int(math.floor((lat + 80) / 8)))
    col = (zone - 1) % 3 * 8 + int(x // 100000) - 1
    row = (int(y // 100000) + (5 if zone % 2 == 0 else 0)) % 20
    scale = 10 ** (5 - digits)
    e, n = int((x % 100000) // scale), int((y % 100000) // scale)
    return "%02d%s %s%s %0*d %0*d" % (zone, "CDEFGHJKLMNPQRSTUVWX"[band], "ABCDEFGHJKLMNPQRSTUVWXYZ"[col],
                                      "ABCDEFGHJKLMNPQRSTUV"[row], digits, e, digits, n)


# ------------------------------------------------------------------------------------------------------------------
# Settings
# ------------------------------------------------------------------------------------------------------------------
DEFAULTS = {
    "MAVLINK_PORT": "COM3",          # the radio's COM port, or a pymavlink connection string such as udpin:127.0.0.1:14560
    "MAVLINK_BAUD": "57600",         # RFD900 / SiK default; ignored for udp
    "MAV_SYSID": "",                 # empty = use the first vehicle heard
    "MAV_REQUEST_STREAMS": "0",      # 1 = send a GCS heartbeat and ask the vehicle for position streams (transmits to the vehicle)
    "CALLSIGN": "Mayman",
    "UID": "",                       # empty = same as the callsign
    "COT_TYPE": "a-f-A-M-H-Q",       # friendly / air / military / rotary-wing / UAV. Fixed wing UAV: a-f-A-M-F-Q
    "TEAM": "Cyan",
    "ROLE": "Team Member",
    "COT_URL": "tls://tak.example.com:8089",
    "UPDATE_SECONDS": "1.0",
    "STALE_SECONDS": "30",
    "MAX_TELEMETRY_AGE": "5",        # stop sending if the last position is older than this (the marker then goes stale)
    "REMARKS_MGRS_DIGITS": "5",
    "TLS_CLIENT_CERT": "",           # .p12 client certificate from your TAK data package
    "TLS_CLIENT_CAFILE": "",         # PEM file with the server's CA (see convert_truststore.py)
    "TLS_DONT_CHECK_HOSTNAME": "0",  # 1 if the server certificate's name does not match the address you connect to
    "TLS_DONT_VERIFY": "0",          # 1 switches server verification OFF. Leave 0 except for a quick test
}


def load_settings(path):
    cp = configparser.ConfigParser(interpolation=None)
    cp.read_dict({SECTION: DEFAULTS})
    if path:
        if not os.path.exists(path):
            sys.exit("Config file not found: %s" % path)
        cp.read(path, encoding="utf-8")
    return cp[SECTION]


def truthy(v):
    return str(v).strip().lower() in ("1", "true", "yes", "on")


def pytak_config(s):
    """Translate the friendly settings into PyTAK's settings (a SectionProxy, as PyTAK expects)."""
    d = {
        "COT_URL": s["COT_URL"],
        "COT_STALE": str(int(float(s["STALE_SECONDS"]))),
        "PYTAK_NO_HELLO": "1",
    }
    url = s["COT_URL"].lower()
    if url.startswith(("tls:", "ssl:")):
        cert = s.get("TLS_CLIENT_CERT", "")
        if not cert:
            sys.exit("COT_URL is a TLS address, so TLS_CLIENT_CERT (your .p12 client certificate) must be set in the config.")
        if not os.path.exists(cert):
            sys.exit("Client certificate not found: %s" % cert)
        d["PYTAK_TLS_CLIENT_CERT"] = cert
        cafile = s.get("TLS_CLIENT_CAFILE", "")
        if cafile:
            if not os.path.exists(cafile):
                sys.exit("CA file not found: %s (make it with convert_truststore.py)" % cafile)
            d["PYTAK_TLS_CLIENT_CAFILE"] = cafile
        pw = os.environ.get("TAK_CERT_PASSWORD")
        if pw is None:
            pw = getpass.getpass("Password for %s (typed here, not saved): " % os.path.basename(cert))
        d["PYTAK_TLS_CLIENT_PASSWORD"] = pw
        if truthy(s.get("TLS_DONT_CHECK_HOSTNAME", "0")):
            d["PYTAK_TLS_DONT_CHECK_HOSTNAME"] = "1"
        if truthy(s.get("TLS_DONT_VERIFY", "0")):
            d["PYTAK_TLS_DONT_VERIFY"] = "1"
    cp = configparser.ConfigParser(interpolation=None)
    cp.read_dict({SECTION: d})
    return cp[SECTION]


# ------------------------------------------------------------------------------------------------------------------
# MAVLink side
# ------------------------------------------------------------------------------------------------------------------
class Telemetry:
    """Latest values from the vehicle, filled in by the reader thread and read by the sender."""

    def __init__(self, want_sysid=None):
        self.lock = threading.Lock()
        self.want_sysid = want_sysid
        self.sysid = want_sysid
        self.d = {}
        self.t_pos = 0.0
        self.t_hb = 0.0
        self.connected = False
        self.message_count = 0

    def update(self, msg):
        ty = msg.get_type()
        src = msg.get_srcSystem()
        with self.lock:
            if ty == "HEARTBEAT":
                if msg.type in NOT_A_VEHICLE or msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                    return
                if self.sysid is None:
                    self.sysid = src
                    LOG.info("vehicle found: system %d, MAVLink type %d", src, msg.type)
                if src != self.sysid:
                    return
                self.d["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                try:
                    self.d["mode"] = mavutil.mode_string_v10(msg)
                except Exception:
                    self.d["mode"] = "?"
                self.d["vtype"] = msg.type
                self.t_hb = time.monotonic()
                self.message_count += 1
                return
            if self.sysid is None or src != self.sysid:
                return
            self.message_count += 1
            if ty == "GLOBAL_POSITION_INT":
                if msg.lat == 0 and msg.lon == 0:
                    return
                self.d.update(lat=msg.lat / 1e7, lon=msg.lon / 1e7, alt_msl=msg.alt / 1000.0, rel_alt=msg.relative_alt / 1000.0,
                              vx=msg.vx / 100.0, vy=msg.vy / 100.0, vz=msg.vz / 100.0,
                              hdg=(msg.hdg / 100.0) if msg.hdg != 65535 else None)
                self.t_pos = time.monotonic()
            elif ty == "GPS_RAW_INT":
                self.d.update(fix=msg.fix_type, sats=msg.satellites_visible, hdop=(msg.eph / 100.0) if msg.eph != 65535 else None)
                ell = getattr(msg, "alt_ellipsoid", 0)
                if ell:
                    self.d["alt_hae"] = ell / 1000.0
                h_acc = getattr(msg, "h_acc", 0)
                v_acc = getattr(msg, "v_acc", 0)
                if h_acc:
                    self.d["h_acc"] = h_acc / 1000.0
                if v_acc:
                    self.d["v_acc"] = v_acc / 1000.0
                if msg.cog != 65535:
                    self.d["cog"] = msg.cog / 100.0
            elif ty == "VFR_HUD":
                self.d.update(gs=msg.groundspeed, hud_hdg=msg.heading)
            elif ty == "SYS_STATUS":
                self.d["volt"] = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
                self.d["batt_pct"] = msg.battery_remaining if msg.battery_remaining >= 0 else None

    def snapshot(self):
        with self.lock:
            return dict(self.d), self.t_pos, self.t_hb, self.sysid


def reader_thread(settings, tel, stop):
    """Read MAVLink forever; reconnect if the radio is unplugged and plugged back in."""
    port = settings["MAVLINK_PORT"]
    baud = int(settings["MAVLINK_BAUD"])
    request = truthy(settings["MAV_REQUEST_STREAMS"])
    while not stop.is_set():
        try:
            LOG.info("opening MAVLink on %s at %d baud ...", port, baud)
            conn = mavutil.mavlink_connection(port, baud=baud, source_system=255, source_component=190)
            tel.connected = True
            last_hb = 0.0
            while not stop.is_set():
                msg = conn.recv_match(blocking=True, timeout=1)
                if request and time.monotonic() - last_hb > 1.0 and tel.sysid is not None:
                    last_hb = time.monotonic()
                    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    conn.mav.request_data_stream_send(tel.sysid, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
                if msg is not None and msg.get_type() != "BAD_DATA":
                    tel.update(msg)
        except Exception as exc:  # serial port missing, unplugged, busy, ...
            tel.connected = False
            LOG.error("MAVLink connection problem: %s. Trying again in 5 s", exc)
            stop.wait(5)


# ------------------------------------------------------------------------------------------------------------------
# CoT side
# ------------------------------------------------------------------------------------------------------------------
def iso(t):
    return t.strftime("%Y-%m-%dT%H:%M:%S.") + "%03dZ" % (t.microsecond // 1000)


def build_cot(settings, d, now=None):
    """The CoT event (bytes) for the vehicle, or None if there is no usable position yet."""
    if "lat" not in d or "lon" not in d:
        return None
    now = now or dt.datetime.now(dt.timezone.utc)
    stale = now + dt.timedelta(seconds=float(settings["STALE_SECONDS"]))
    callsign = settings["CALLSIGN"]
    uid = settings["UID"] or callsign
    hae = d.get("alt_hae", d.get("alt_msl", 0.0))
    ce = d.get("h_acc") or ((d["hdop"] * 5.0) if d.get("hdop") else 9999999.0)
    le = d.get("v_acc") or 9999999.0
    gs = d.get("gs")
    if gs is None:
        gs = math.hypot(d.get("vx", 0.0), d.get("vy", 0.0))
    course = d.get("cog")
    if course is None or gs < 0.5:
        course = d.get("hdg") if d.get("hdg") is not None else d.get("hud_hdg", 0.0)
    mg = to_mgrs(d["lat"], d["lon"], int(settings["REMARKS_MGRS_DIGITS"]))
    bits = []
    if mg:
        bits.append("MGRS " + mg)
    bits.append("%s %s" % (d.get("mode", "?"), "ARMED" if d.get("armed") else "DISARMED"))
    if "rel_alt" in d:
        bits.append("AGL %.0f m" % d["rel_alt"])
    bits.append("GS %.1f m/s (%.0f kt)" % (gs, gs * M_PER_S_TO_KT))
    if d.get("volt") is not None:
        bits.append("Batt %.1f V%s" % (d["volt"], (" %d%%" % d["batt_pct"]) if d.get("batt_pct") is not None else ""))
    if d.get("sats") is not None:
        bits.append("Sats %d" % d["sats"])
    remarks = " | ".join(bits)
    battery = ('<status battery="%d"/>' % d["batt_pct"]) if d.get("batt_pct") is not None else ""
    xml = (
        '<event version="2.0" uid=%s type=%s how="m-g" time="%s" start="%s" stale="%s">'
        '<point lat="%.7f" lon="%.7f" hae="%.1f" ce="%.1f" le="%.1f"/>'
        '<detail><contact callsign=%s/><__group name=%s role=%s/>'
        '<precisionlocation geopointsrc="GPS" altsrc="GPS"/>'
        '<track course="%.1f" speed="%.2f"/>%s<remarks>%s</remarks></detail></event>'
    ) % (quoteattr(uid), quoteattr(settings["COT_TYPE"]), iso(now), iso(now), iso(stale),
         d["lat"], d["lon"], hae, ce, le, quoteattr(callsign), quoteattr(settings["TEAM"]), quoteattr(settings["ROLE"]),
         course, gs, battery, escape(remarks))
    return xml.encode("utf-8")


class MavSender(pytak.QueueWorker):
    """Puts the vehicle's CoT event on PyTAK's send queue every UPDATE_SECONDS while the telemetry is fresh."""

    def __init__(self, queue, config, settings, tel):
        super().__init__(queue, config)
        self.settings = settings
        self.tel = tel
        self.sent = 0
        self.last_report = 0.0
        self.was_fresh = None

    async def handle_data(self, data):
        await self.put_queue(data)

    async def run(self, _=-1):
        period = max(0.2, float(self.settings["UPDATE_SECONDS"]))
        max_age = float(self.settings["MAX_TELEMETRY_AGE"])
        while True:
            d, t_pos, t_hb, sysid = self.tel.snapshot()
            fresh = t_pos > 0 and (time.monotonic() - t_pos) <= max_age
            if fresh != self.was_fresh:
                LOG.info("telemetry %s", "flowing: sending %s to TAK" % self.settings["CALLSIGN"] if fresh else
                         "not fresh (waiting for a vehicle position): not sending")
                self.was_fresh = fresh
            if fresh:
                event = build_cot(self.settings, d)
                if event:
                    await self.handle_data(event)
                    self.sent += 1
            if time.monotonic() - self.last_report > 15:
                self.last_report = time.monotonic()
                LOG.info("status: sent %d events, MAVLink messages seen %d, vehicle system %s, lat/lon %s",
                         self.sent, self.tel.message_count, sysid,
                         ("%.5f, %.5f" % (d["lat"], d["lon"])) if "lat" in d else "none yet")
            await asyncio.sleep(period)


async def amain(settings, tel):
    cfg = pytak_config(settings)
    clitool = pytak.CLITool(cfg)
    await clitool.setup()
    clitool.add_task(MavSender(clitool.tx_queue, cfg, settings, tel))
    LOG.info("sending to %s as %s (%s)", pytak.sanitize_url_credentials(settings["COT_URL"]) if hasattr(pytak, "sanitize_url_credentials") else settings["COT_URL"],
             settings["CALLSIGN"], settings["COT_TYPE"])
    await clitool.run()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.ini", help="settings file (default config.ini)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this many seconds (for tests)")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    settings = load_settings(args.config)
    sysid = int(settings["MAV_SYSID"]) if settings["MAV_SYSID"].strip() else None
    tel = Telemetry(sysid)
    stop = threading.Event()
    threading.Thread(target=reader_thread, args=(settings, tel, stop), daemon=True).start()

    async def runner():
        task = asyncio.ensure_future(amain(settings, tel))
        if args.seconds:
            await asyncio.sleep(args.seconds)
            task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    try:
        asyncio.run(runner())
    except KeyboardInterrupt:
        LOG.info("stopped")
    finally:
        stop.set()


if __name__ == "__main__":
    main()
