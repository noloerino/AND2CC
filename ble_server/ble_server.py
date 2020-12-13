#!/usr/bin/env python3

import asyncio
import struct
import time
from bleak import BleakClient, BleakScanner

# parser = argparse.ArgumentParser(description='Print advertisement data from a BLE device')
# parser.add_argument('addr', metavar='A', type=str, help='Address of the form XX:XX:XX:XX:XX:XX')
# args = parser.parse_args()
# addr = args.addr.lower()
# if len(addr) != 17:
#     raise ValueError("Invalid address supplied")

DDD_SERVICE_UUID   = "32e61089-2b22-4db5-a914-43ce41986c70"
DDD_REQ_CHAR_UUID  = "32e6108b-2b22-4db5-a914-43ce41986c70"
DDD_RESP_CHAR_UUID = "32e6108c-2b22-4db5-a914-43ce41986c70"

CONN_TIMEOUT = 20.0 # seconds

# < means little endian
# L means unsigned long (4B), B means unsigned byte
REQ_LAYOUT = "<LLBBB"
RESP_LAYOUT = "<LBBH"

class Sync:
    # Requests
    PREPARE = 1
    COMMIT = 2
    ABORT = 3
    # Responses
    VOTE_COMMIT = 1
    VOTE_ABORT = 2
    ACK = 3

CMD_LUT = {
    "on": 1,
    "off": 2,
    "l": 3,
    "r": 4,
    "f": 5,
    "b": 6,
    "z": 7,
}

PREPARE_TARGET_DELAY_MS = 2000

class Channels:
    def __init__(self, i, buckler, pair):
        self.id = i
        self.buckler = buckler
        self.req_ch = pair[0] 
        self.resp_ch = pair[1]

    async def prepare(self, seq_no, t1, cmd_id):
        i = self.id
        print(f"# {i} sending 2PC prepare [t1={t1}]")
        # PTP/2PC from leader to follower
        await self.buckler.write_gatt_char(
            self.req_ch,
            # Prepare:
            # - t1 [ms]: u32
            # - target offset from t1 [ms]: u32
            # - 2PC command: u8
            # - robot command: u8
            # - seq_no: u8
            struct.pack(
                REQ_LAYOUT,
                t1,
                PREPARE_TARGET_DELAY_MS,
                Sync.PREPARE,
                cmd_id,
                seq_no
            ),
            response=True
        )
        # PTP/2PC from follower to leader
        t2, sync_resp_id, resp_seq_no, _ = struct.unpack(
            RESP_LAYOUT,
            await self.buckler.read_gatt_char(self.resp_ch)
        )
        if resp_seq_no != seq_no:
            print(f"# {i} unexpected sequence number: got {resp_seq_no}, exp {seq_no}")
            return False
        if sync_resp_id == Sync.VOTE_COMMIT:
            print(f"# {i} voted commit [t2={t2}]")
            return True
        elif sync_resp_id == Sync.VOTE_ABORT:
            print(f"# {i} voted abort")
            return False
        else:
            print(f"# {i} voted invalid ({sync_resp_id})")
            return False

    async def wait_for_ack(self, seq_no):
        _, sync_resp_id, resp_seq_no, _ = struct.unpack(
            RESP_LAYOUT,
            await self.buckler.read_gatt_char(self.resp_ch)
        )
        if resp_seq_no != seq_no:
            print(f"# {self.id} unexpected sequence number: got {resp_seq_no}, exp {seq_no}")
            return False
        return sync_resp_id == Sync.ACK

    async def commit(self, seq_no, t4):
        print(f"# {self.id} sending 2PC commit")
        await self.buckler.write_gatt_char(
            self.req_ch,
            # Commit:
            # - t4 [ms]: u32
            # - _: u32
            # - 2PC command: u8
            # - _: u8
            # - seq_no: u8
            struct.pack(REQ_LAYOUT, t4, 0, Sync.COMMIT, 0, seq_no),
            response=True
        )
        return await self.wait_for_ack(seq_no)

    async def abort(self, seq_no):
        print(f"# {self.id} sending 2PC abort")
        await self.buckler.write_gatt_char(
            self.req_ch,
            struct.pack(REQ_LAYOUT, 0, 0, Sync.ABORT, 0),
            response=False
        )
        return await self.wait_for_ack(seq_no)

async def get_buckler_ch(buckler):
    await buckler.connect(timeout=CONN_TIMEOUT)
    # Get service
    svs = await buckler.get_services()
    sv = svs.get_service(DDD_SERVICE_UUID)
    # Get characteristic
    return (
        sv.get_characteristic(DDD_REQ_CHAR_UUID).handle,
        sv.get_characteristic(DDD_RESP_CHAR_UUID).handle,
    )

# https://bleak.readthedocs.io/en/latest/usage.html
async def run():
    global led_status, l_drive, r_drive
    print("searching for DDDs...")
    buckler_0 = None
    buckler_1 = None
    while buckler_0 is None:
    # while buckler_0 is None or buckler_1 is None:
        devices = await BleakScanner.discover()
        for d in devices:
            # print(f"device {d} at address {d.address}")
            if "EE149" in d.name:
                print(f"discovered device {d}")
                if buckler_0 is None and "(0)" in d.name:
                    buckler_0 = BleakClient(d.address)
                    print(f"discovered DDD 0 @ addr {d.address}")
                    break
        #             if buckler_1 is not None:
        #                 break
        #         if buckler_1 is None and "(1)" in d.name:
        #             buckler_1 = BleakClient(d.address)
        #             print(f"discovered DDD 1 @ addr {d.address}")
        #             if buckler_0 is not None:
        #                 break
        # if buckler_0 is None or buckler_1 is None:
        #     print("retrying discovery...")

    try:
        print("DDDs found, verifying characteristics...")
        channel_pairs = await asyncio.gather(
            get_buckler_ch(buckler_0),
            # get_buckler_ch(buckler_1)
        )
        ch_0 = Channels(0, buckler_0, channel_pairs[0])
        # ch_1 = Channels(1, buckler_1, channel_pairs[1])
        print("Both DDDs ready!")

        seq_no = 0
        while True:
            seq_no += 1
            cmd = input("ddd> ").strip()
            if cmd == "q":
                print("quitting")
                break
            if cmd in CMD_LUT:
                print("# Beginning 2PC sequence")
                cmd_id = CMD_LUT[cmd]
                t1 = int(time.process_time() * 1000)
                # [p1, p2] = await asyncio.gather(
                #     ch_0.prepare(t1, cmd_id),
                #     ch_1.prepare(t1, cmd_id),
                # )
                p1 = await ch_0.prepare(seq_no, t1, cmd_id)
                if not p1:
                    print("# Aborting 2PC")
                    await ch_0.abort(seq_no)
                    continue
                t4 = int(time.process_time() * 1000)
                c1 = await ch_0.commit(seq_no, t4)
                if not c1:
                    print("# Didn't get 2PC ACK, oh well")
            else:
                print(f"invalid command: {cmd}")
    finally:
        await asyncio.gather(buckler_0.disconnect(), buckler_1.disconnect())

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
