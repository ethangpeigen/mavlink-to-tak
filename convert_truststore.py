#!/usr/bin/env python3
"""Turn a TAK truststore (.p12) into a PEM file of CA certificates, which is what the bridge needs for TLS.

    python convert_truststore.py certs\\truststore-root.p12 certs\\ca.pem

It asks for the truststore's password (typed here, not saved or shown). Only the public CA certificates are written.
The client certificate (.p12) is used as it is and needs no conversion.
"""
import getpass
import os
import sys

from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.serialization import pkcs12


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    src, dst = sys.argv[1], sys.argv[2]
    if not os.path.exists(src):
        sys.exit("File not found: %s" % src)
    pw = os.environ.get("TAK_TRUSTSTORE_PASSWORD")
    if pw is None:
        pw = getpass.getpass("Password for %s: " % os.path.basename(src))
    data = open(src, "rb").read()
    certs = []
    try:
        p12 = pkcs12.load_pkcs12(data, pw.encode("utf-8"))
        if p12.cert:
            certs.append(p12.cert.certificate)
        certs += [c.certificate for c in p12.additional_certs]
    except Exception as exc:
        sys.exit("Could not read %s (wrong password, or not a PKCS#12 file): %s" % (src, exc))
    if not certs:
        sys.exit("No certificates found in %s" % src)
    with open(dst, "wb") as f:
        for c in certs:
            f.write(c.public_bytes(serialization.Encoding.PEM))
    for c in certs:
        print("wrote:", c.subject.rfc4514_string(), "(expires %s)" % c.not_valid_after_utc.date())
    print("-> %s (%d certificate(s))" % (dst, len(certs)))


if __name__ == "__main__":
    main()
