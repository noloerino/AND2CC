#!/usr/bin/env python3

# import struct
# from bluepy.btle import Peripheral, DefaultDelegate
import asyncio
from bleak import BleakClient, BleakScanner
# import argparse

# parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
# parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
# args = parser.parse_args()
# addr = args.addr.lower()
# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")

DDD_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
DDD_CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"

# https://bleak.readthedocs.io/en/latest/usage.html
async def run():
    addr = None
    print("searching for buckler...")
    devices = await BleakScanner.discover()
    for d in devices:
        # print(f"device {d} at address {d.address}")
        if "EE149" in d.name:
            print(f"device {d}")
            addr = d.address
            break
    if addr is None:
        print("couldn't find buckler")
        return

    # addr = "C2BFB6D8-9256-43F1-8E79-5CC96EEE04F6"

    buckler = BleakClient(addr)
    try:
        print(f"connecting to addr {addr}")
        await buckler.connect(timeout=20.0)
        print("connected")

        # Get service
        svs = await buckler.get_services()
        sv = svs.get_service(DDD_SERVICE_UUID)
        # Get characteristic
        ch = sv.get_characteristic(DDD_CHAR_UUID).handle

        while True:
            char_value = await buckler.read_gatt_char(ch)
            print(f"read value: {char_value}")
            display = input("Type a little-endian hex number to send as a byte array> ")
            await buckler.write_gatt_char(ch, int(display, 16).to_bytes(3, byteorder="little"))
    finally:
        await buckler.disconnect()

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
