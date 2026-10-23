#!/usr/bin/env python3
"""End-to-end tests for mavlink_to_tak.py. No radio, no TAK Server and no real certificates needed.

    python test_bridge.py

1. MGRS maths against the 'mgrs' reference library (pip install mgrs).
2. Bridge -> plain TCP "server": a simulated vehicle (sim_mavlink.py) is read over UDP and CoT events must arrive.
3. Bridge -> TLS "server" that REQUIRES a client certificate: test CA, server and client certificates are generated here,
   the client certificate is a password-protected .p12, the truststore is a Java-style .p12 converted with
   convert_truststore.py. Events must arrive over TLS and the server must have seen the client certificate.
4. A wrong certificate password must fail with a clear message and send nothing.
"""
import datetime as dt
import os
import random
import re
import socket
import ssl
import subprocess
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from cryptography import x509
from cryptography.hazmat.primitives import hashes, serialization
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives.serialization import pkcs12
from cryptography.x509.oid import NameOID

HERE = os.path.dirname(os.path.abspath(__file__))
PY = sys.executable
FAILS = []


def check(name, ok, detail=""):
    print("  [%s] %s%s" % ("ok" if ok else "FAIL", name, (" - " + detail) if detail else ""))
    if not ok:
        FAILS.append(name)


# ------------------------------------------------------------------ 1. MGRS
def test_mgrs():
    import mgrs
    sys.path.insert(0, HERE)
    from mavlink_to_tak import to_mgrs
    m = mgrs.MGRS()
    rng = random.Random(11)
    pts = [(rng.uniform(-79.9, 83.9), rng.uniform(-180, 179.99)) for _ in range(3000)]
    pts += [(60.0, 5.0), (73.0, 5.0), (73.0, 15.0), (73.0, 25.0), (73.0, 36.0), (0.0, 0.0), (-33.9, 151.2), (38.8895, -77.0353), (51.5, -0.12)]
    exact = edge = bad = 0
    for digits in (3, 4, 5):
        for lat, lon in pts:
            got = to_mgrs(lat, lon, digits)
            ref = m.toMGRS(lat, lon, MGRSPrecision=digits)
            zb = ref[:3] if ref[:2].isdigit() else ref[:2]
            nums = ref[len(zb) + 2:]
            half = len(nums) // 2
            exp = "%s %s %s %s" % (zb, ref[len(zb):len(zb) + 2], nums[:half], nums[half:])
            if got == exp:
                exact += 1
            else:
                g, e = got.split(), exp.split()
                if len(g) == 4 and g[:2] == e[:2] and abs(int(g[2]) - int(e[2])) <= 1 and abs(int(g[3]) - int(e[3])) <= 1:
                    edge += 1
                else:
                    bad += 1
                    if bad < 4:
                        print("     mismatch", lat, lon, digits, got, exp)
    check("MGRS matches the reference library", bad == 0, "%d exact, %d off by 1 at a cell edge, %d wrong" % (exact, edge, bad))


# ------------------------------------------------------------------ helpers
class Server:
    """A tiny CoT receiver (plain or TLS with required client certificate)."""

    def __init__(self, ctx=None):
        self.ctx = ctx
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(2)
        self.port = self.sock.getsockname()[1]
        self.data = b""
        self.peer_subject = None
        self.err = None
        threading.Thread(target=self.loop, daemon=True).start()

    def loop(self):
        while True:
            try:
                c, _ = self.sock.accept()
            except OSError:
                return
            try:
                if self.ctx:
                    c = self.ctx.wrap_socket(c, server_side=True)
                    cert = c.getpeercert()
                    if cert:
                        self.peer_subject = dict(x[0] for x in cert["subject"])
                c.settimeout(1)
                while True:
                    try:
                        chunk = c.recv(65536)
                    except socket.timeout:
                        continue
                    except OSError:
                        break
                    if not chunk:
                        break
                    self.data += chunk
            except Exception as exc:
                self.err = repr(exc)

    def events(self):
        out = []
        for m in re.finditer(rb"<event .*?</event>", self.data, re.S):
            out.append(ET.fromstring(m.group(0)))
        return out

    def close(self):
        self.sock.close()


def write_cfg(path, **kv):
    base = {
        "MAVLINK_PORT": "udpin:127.0.0.1:%d" % 14600, "CALLSIGN": "Mayman", "UPDATE_SECONDS": "0.5", "STALE_SECONDS": "30",
        "COT_TYPE": "a-f-A-M-H-Q",
    }
    base.update(kv)
    with open(path, "w") as f:
        f.write("[mavlink_to_tak]\n" + "".join("%s = %s\n" % (k, v) for k, v in base.items()))


def run_bridge(cfg, seconds, env=None, sim_port=14600):
    sim = subprocess.Popen([PY, os.path.join(HERE, "sim_mavlink.py"), "--port", str(sim_port), "--seconds", str(seconds + 2)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1.0)
    e = dict(os.environ)
    e.update(env or {})
    p = subprocess.run([PY, os.path.join(HERE, "mavlink_to_tak.py"), "--config", cfg, "--seconds", str(seconds)], capture_output=True, text=True, env=e, timeout=seconds + 40)
    sim.terminate()
    return p


def check_events(evs, label):
    check(label + ": events arrived", len(evs) >= 4, "%d events" % len(evs))
    if not evs:
        return
    e = evs[-1]
    pt = e.find("point")
    contact = e.find("detail/contact")
    check(label + ": callsign is Mayman", contact is not None and contact.get("callsign") == "Mayman")
    check(label + ": type a-f-A-M-H-Q", e.get("type") == "a-f-A-M-H-Q")
    lat, lon = float(pt.get("lat")), float(pt.get("lon"))
    check(label + ": position is the simulated one", abs(lat - 38.8895) < 0.001 and abs(lon + 77.0353) < 0.001, "%.5f, %.5f" % (lat, lon))
    rem = e.find("detail/remarks").text
    check(label + ": remarks carry MGRS, mode and arming", "MGRS 18S UJ" in rem and "LOITER" in rem.upper() and "ARMED" in rem, rem[:90])
    check(label + ": time / stale are valid", e.get("stale") > e.get("time"))


# ------------------------------------------------------------------ certificates
def make_certs(d, password):
    def key():
        return rsa.generate_private_key(public_exponent=65537, key_size=2048)

    def name(cn):
        return x509.Name([x509.NameAttribute(NameOID.COMMON_NAME, cn)])

    now = dt.datetime.now(dt.timezone.utc)
    ca_key = key()
    ca = (x509.CertificateBuilder().subject_name(name("Test CA")).issuer_name(name("Test CA")).public_key(ca_key.public_key())
          .serial_number(x509.random_serial_number()).not_valid_before(now - dt.timedelta(days=1)).not_valid_after(now + dt.timedelta(days=30))
          .add_extension(x509.BasicConstraints(ca=True, path_length=None), critical=True).sign(ca_key, hashes.SHA256()))

    def leaf(cn, san=None):
        k = key()
        b = (x509.CertificateBuilder().subject_name(name(cn)).issuer_name(ca.subject).public_key(k.public_key())
             .serial_number(x509.random_serial_number()).not_valid_before(now - dt.timedelta(days=1)).not_valid_after(now + dt.timedelta(days=30)))
        if san:
            b = b.add_extension(x509.SubjectAlternativeName(san), critical=False)
        return k, b.sign(ca_key, hashes.SHA256())

    import ipaddress
    sk, sc = leaf("localhost", [x509.DNSName("localhost"), x509.IPAddress(ipaddress.ip_address("127.0.0.1"))])
    ck, cc = leaf("mayman-client")
    pem = serialization.Encoding.PEM
    open(os.path.join(d, "server.pem"), "wb").write(sc.public_bytes(pem))
    open(os.path.join(d, "server.key"), "wb").write(sk.private_bytes(pem, serialization.PrivateFormat.PKCS8, serialization.NoEncryption()))
    open(os.path.join(d, "ca_for_server.pem"), "wb").write(ca.public_bytes(pem))
    enc = serialization.BestAvailableEncryption(password.encode())
    open(os.path.join(d, "client.p12"), "wb").write(pkcs12.serialize_key_and_certificates(b"client", ck, cc, [ca], enc))
    entries = [pkcs12.PKCS12Certificate(ca, b"root")]
    open(os.path.join(d, "truststore-root.p12"), "wb").write(pkcs12.serialize_java_truststore(entries, enc))


# ------------------------------------------------------------------ 2, 3, 4
def test_plain_tcp(tmp):
    srv = Server()
    cfg = os.path.join(tmp, "plain.ini")
    write_cfg(cfg, COT_URL="tcp://127.0.0.1:%d" % srv.port)
    p = run_bridge(cfg, 8)
    check_events(srv.events(), "plain TCP")
    check("plain TCP: bridge exited cleanly", p.returncode == 0, (p.stderr or "")[-200:])
    srv.close()


def test_tls(tmp):
    pw = "t3st-" + str(random.randint(100000, 999999))
    make_certs(tmp, pw)
    ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    ctx.load_cert_chain(os.path.join(tmp, "server.pem"), os.path.join(tmp, "server.key"))
    ctx.verify_mode = ssl.CERT_REQUIRED
    ctx.load_verify_locations(os.path.join(tmp, "ca_for_server.pem"))
    srv = Server(ctx)
    ca_pem = os.path.join(tmp, "ca.pem")
    r = subprocess.run([PY, os.path.join(HERE, "convert_truststore.py"), os.path.join(tmp, "truststore-root.p12"), ca_pem],
                       capture_output=True, text=True, env=dict(os.environ, TAK_TRUSTSTORE_PASSWORD=pw))
    check("truststore .p12 converts to a CA PEM", r.returncode == 0 and os.path.exists(ca_pem), (r.stdout + r.stderr).strip()[:150])
    cfg = os.path.join(tmp, "tls.ini")
    write_cfg(cfg, COT_URL="tls://127.0.0.1:%d" % srv.port, TLS_CLIENT_CERT=os.path.join(tmp, "client.p12"), TLS_CLIENT_CAFILE=ca_pem)
    p = run_bridge(cfg, 9, env={"TAK_CERT_PASSWORD": pw}, sim_port=14601 if False else 14600)
    check_events(srv.events(), "TLS")
    check("TLS: server saw the client certificate", (srv.peer_subject or {}).get("commonName") == "mayman-client", str(srv.peer_subject))
    check("TLS: bridge exited cleanly", p.returncode == 0, (p.stderr or "")[-300:])
    srv.close()
    # 4. wrong password
    srv2 = Server(ctx)
    cfg2 = os.path.join(tmp, "tls2.ini")
    write_cfg(cfg2, COT_URL="tls://127.0.0.1:%d" % srv2.port, TLS_CLIENT_CERT=os.path.join(tmp, "client.p12"), TLS_CLIENT_CAFILE=ca_pem)
    p2 = run_bridge(cfg2, 6, env={"TAK_CERT_PASSWORD": "wrong-password"})
    out = (p2.stdout or "") + (p2.stderr or "")
    check("wrong password: nothing is sent", len(srv2.events()) == 0, "%d events" % len(srv2.events()))
    check("wrong password: clear message", re.search(r"Password|password|Error opening resource", out) is not None, out.strip().splitlines()[-1][:150] if out.strip() else "no output")
    srv2.close()


if __name__ == "__main__":
    print("== MGRS")
    test_mgrs()
    with tempfile.TemporaryDirectory() as tmp:
        print("== bridge over plain TCP")
        test_plain_tcp(tmp)
        print("== bridge over TLS with a client certificate")
        test_tls(tmp)
    print("\nRESULT:", "all passed" if not FAILS else "FAILED: " + ", ".join(FAILS))
    sys.exit(1 if FAILS else 0)
