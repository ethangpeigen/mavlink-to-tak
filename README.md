# MAVLink to TAK bridge

Reads telemetry from a drone over an RFD900 (SiK) radio on a Windows COM port and sends it to a TAK Server as a
Cursor on Target track named **Mayman**. WinTAK, ATAK and iTAK connected to that server will see it.

SiKW00F/AryaOS is not needed. It is a counter-drone eavesdropping toolkit for Linux; this bridge reads your own link directly.

## Setup at home

1. Install Python 3.10+ from python.org (tick "Add python.exe to PATH").
2. In this folder:
   ```
   pip install -r requirements.txt
   ```
3. Put the files from your TAK data package in `certs\`: the client certificate (`client.p12`) and the truststore (`truststore-root.p12`).
4. Convert the truststore to a PEM (asks for the truststore password; nothing is saved):
   ```
   python convert_truststore.py certs\truststore-root.p12 certs\ca.pem
   ```
5. Copy `config.example.ini` to `config.ini`. Set `MAVLINK_PORT` (Device Manager, Ports, for example `COM7`) and `COT_URL`
   (`tls://your-server:8089`).
6. Plug in the radio, power the drone, then run:
   ```
   python mavlink_to_tak.py --config config.ini
   ```
   It asks for the client certificate password (or set the environment variable `TAK_CERT_PASSWORD`).
7. Within a few seconds the log says `telemetry flowing: sending Mayman to TAK` and the marker appears in WinTAK.

## Try it with no radio

```
python sim_mavlink.py --port 14560
```
and set `MAVLINK_PORT = udpin:127.0.0.1:14560` in a second config. `python test_bridge.py` runs the full automatic test
(MGRS maths, plain TCP, TLS with a client certificate, wrong password) with generated test certificates.

## Notes and troubleshooting

- **One program per COM port.** Close Mission Planner/QGC while the bridge runs, or split the radio with MAVProxy or mavlink-router.
- The bridge only listens by default. ArduPilot normally streams position on its own; if nothing arrives, set `MAV_REQUEST_STREAMS = 1`.
- No marker: the log prints every 15 s how many MAVLink messages arrived and whether there is a GPS fix.
- TLS name mismatch (connecting by IP): `TLS_DONT_CHECK_HOSTNAME = 1`. Do not use `TLS_DONT_VERIFY` except for a quick test.
- If the radio drops out, the marker stops updating and goes stale after `STALE_SECONDS`.
- The remarks field carries MGRS, flight mode, armed state, height, speed, battery and satellites.

## Status

Tested against a simulated vehicle and local test servers only. Not yet tried with a real RFD900 or the real TAK Server.
