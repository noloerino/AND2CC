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

CONN_TIMEOUT = 20.0 # seconds

# < means little endian
# B means unsigned byte
STATE_LAYOUT = "<B"

CMD_LUT = {
    "on": 0,
    "off": 1,
    "l": 2,
    "r": 3,
    "f": 4,
    "b": 5,
    "z": 6,
}

async def get_buckler_ch(buckler):
    await buckler.connect(timeout=CONN_TIMEOUT)
    # Get service
    svs = await buckler.get_services()
    sv = svs.get_service(DDD_SERVICE_UUID)
    # Get characteristic
    return sv.get_characteristic(DDD_CHAR_UUID).handle

# https://bleak.readthedocs.io/en/latest/usage.html
async def run():
    global led_status, l_drive, r_drive
    print("searching for DDDs...")
    buckler_0 = None
    buckler_1 = None
    task_0 = None
    task_1 = None
    while buckler_0 is None or buckler_1 is None:
        devices = await BleakScanner.discover()
        for d in devices:
            # print(f"device {d} at address {d.address}")
            if "EE149" in d.name:
                print(f"discovered device {d}")
                if buckler_0 is None and "(0)" in d.name:
                    buckler_0 = BleakClient(d.address)
                    print(f"discovered DDD 0 @ addr {d.address}")
                    if buckler_1 is not None:
                        break
                if buckler_1 is None and "(1)" in d.name:
                    buckler_1 = BleakClient(d.address)
                    print(f"discovered DDD 1 @ addr {d.address}")
                    if buckler_0 is not None:
                        break
        if buckler_0 is None or buckler_1 is None:
            print("retrying discovery...")

    try:
        print("DDDs found, verifying characteristics...")
        chs = await asyncio.gather(get_buckler_ch(buckler_0), get_buckler_ch(buckler_1))
        print("Both DDDs ready!")

        while True:
            # char_values = await asyncio.gather(
            #     buckler_0.read_gatt_char(chs[0]),
            #     buckler_1.read_gatt_char(chs[1]),
            # )
            # print(f"read value: {[struct.unpack(STATE_LAYOUT, v) for v in char_values]}")
            cmd = input("ddd> ").strip()
            if cmd == "q":
                print("quitting")
                break
            if cmd in CMD_LUT:
                await asyncio.gather(
                    buckler_0.write_gatt_char(chs[0], struct.pack(STATE_LAYOUT, CMD_LUT[cmd])),
                    buckler_1.write_gatt_char(chs[1], struct.pack(STATE_LAYOUT, CMD_LUT[cmd])),
                )
            else:
                print(f"invalid command: {cmd}")
    finally:
        await asyncio.gather(buckler_0.disconnect(), buckler_1.disconnect())

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
