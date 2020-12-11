#!/usr/bin/env python3

import struct
import asyncio
from bleak import BleakClient, BleakScanner

# parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
# parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
# args = parser.parse_args()
# addr = args.addr.lower()
# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")

DDD_SERVICE_UUID = "32e61089-2b22-4db5-a914-43ce41986c70"
DDD_CHAR_UUID    = "32e6108a-2b22-4db5-a914-43ce41986c70"

led_status = False
l_drive = 0.0
r_drive = 0.0

# < means little endian
# B means unsigned byte
# f means float
# some of these are just padding fields
STATE_LAYOUT = "<Bbbbff"

SPEED = -100.0

# https://bleak.readthedocs.io/en/latest/usage.html
async def run():
    global led_status, l_drive, r_drive
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
            # TODO use this feedback to set state vars
            print(f"read value: {struct.unpack(STATE_LAYOUT, char_value)}")
            cmd = input("ddd> ").strip()
            if cmd == "on":
                led_status = True
            elif cmd =="off":
                led_status = False
            elif cmd == "l":
                l_drive = SPEED
                r_drive = -SPEED
            elif cmd == "r":
                l_drive = -SPEED
                r_drive = SPEED
            elif cmd == "f":
                l_drive = SPEED
                r_drive = SPEED
            elif cmd == "b":
                l_drive = -SPEED
                r_drive = -SPEED
            elif cmd == "z":
                l_drive = 0
                r_drive = 0
            elif cmd == "quit":
                print("quitting")
                break
            else:
                print(f"invalid command: {cmd}")
                continue
            await buckler.write_gatt_char(ch, struct.pack(
                STATE_LAYOUT, led_status, 0, 0, 0, l_drive, r_drive
            ))
    finally:
        await buckler.disconnect()

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
