#!/usr/bin/env python3
"""Generate a Zigbee2MQTT OTA override index for the PC1001_ZB firmware.

Parses the Zigbee OTA header of the built image to read the real
manufacturerCode / imageType / fileVersion (so the index can never drift from
the firmware), computes the sha512, copies the .ota next to the index, and
writes index.json in the Z2M "override index" format.

Usage:
    python3 ota/gen_ota_index.py                 # auto-pick newest build/*.ota
    python3 ota/gen_ota_index.py build/pc1001_zb_v1.1.0.0.ota
"""
import glob
import hashlib
import json
import os
import shutil
import struct
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OTA_DIR = os.path.join(REPO, "ota")

# Match fields so Z2M only offers this image to the right device.
MODEL_ID = "PC1001_ZB"
MANUFACTURER_NAME = "Custom devices (DiY)"


def parse_ota(path):
    data = open(path, "rb").read()
    magic, _hver, _hlen, _fctrl, mcode, itype, fver, _zstack = struct.unpack(
        "<IHHHHHIH", data[0:20]
    )
    if magic != 0x0BEEF11E:
        raise SystemExit(f"{path}: not a Zigbee OTA file (bad magic 0x{magic:08X})")
    return {
        "manufacturerCode": mcode,
        "imageType": itype,
        "fileVersion": fver,
        "fileSize": len(data),
        "sha512": hashlib.sha512(data).hexdigest(),
        "_data": data,
    }


def main():
    if len(sys.argv) > 1:
        src = sys.argv[1]
    else:
        builds = sorted(
            glob.glob(os.path.join(REPO, "build", "*.ota")), key=os.path.getmtime
        )
        if not builds:
            raise SystemExit("No build/*.ota found - run `idf.py build` first.")
        src = builds[-1]

    info = parse_ota(src)
    fname = os.path.basename(src)

    # Copy the image next to the index so the relative `url` resolves locally.
    dst = os.path.join(OTA_DIR, fname)
    if os.path.abspath(src) != os.path.abspath(dst):
        shutil.copy2(src, dst)

    entry = {
        "fileVersion": info["fileVersion"],
        "fileSize": info["fileSize"],
        "manufacturerCode": info["manufacturerCode"],
        "imageType": info["imageType"],
        "sha512": info["sha512"],
        "url": fname,
        "modelId": MODEL_ID,
        "manufacturerName": [MANUFACTURER_NAME],
    }

    index_path = os.path.join(OTA_DIR, "index.json")
    with open(index_path, "w") as f:
        json.dump([entry], f, indent=2)
        f.write("\n")

    print(f"Image      : {fname}")
    print(f"  version  : {info['fileVersion']} (0x{info['fileVersion']:08X})")
    print(f"  manuf    : {info['manufacturerCode']} (0x{info['manufacturerCode']:04X})")
    print(f"  imageType: {info['imageType']} (0x{info['imageType']:04X})")
    print(f"  size     : {info['fileSize']} bytes")
    print(f"  sha512   : {info['sha512'][:32]}...")
    print(f"Wrote      : {index_path}")


if __name__ == "__main__":
    main()
