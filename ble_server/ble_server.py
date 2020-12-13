#!/usr/bin/env python3

import asyncio
import struct
import random
import time
from bleak import BleakClient, BleakScanner

PREPARE_TARGET_DELAY_MS = 4000

DDD_SERVICE_UUID   = "32e61089-2b22-4db5-a914-43ce41986c70"
DDD_REQ_CHAR_UUID  = "32e6108b-2b22-4db5-a914-43ce41986c70"
DDD_RESP_CHAR_UUID = "32e6108c-2b22-4db5-a914-43ce41986c70"

CONN_TIMEOUT = 20.0 # seconds

# < means little endian
# L means unsigned long (4B), l means signed long, B means unsigned byte
REQ_PREPARE_LAYOUT = "<LLBBB"
REQ_COMMIT_LAYOUT = "<LlBBB"
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

start_ms = int(time.time() * 1000)

def now_ms():
    # Can't use process_time since we're probably sleeping a lot with async
    return int(time.time() * 1000) - start_ms

class Channels:
    def __init__(self, i, buckler, pair):
        self.id = i
        self.buckler = buckler
        self.req_ch = pair[0] 
        self.resp_ch = pair[1]

    async def prepare(self, seq_no, t1, cmd_id):
        """
        Performs a 2PC prepare.

        Returns the rough clock error if the result is valid, or None if the the result was invalid.
        """
        i = self.id
        print(f"# {i} sending 2PC prepare [t1={t1}]")
        # PTP/2PC from leader to follower
        await self.buckler.write_gatt_char(
            self.req_ch,
            # Prepare:
            # - t1: u32
            # - target offset from t1 [ms]: u32
            # - 2PC command: u8
            # - robot command: u8
            # - seq_no: u8
            struct.pack(
                REQ_PREPARE_LAYOUT,
                t1,
                PREPARE_TARGET_DELAY_MS,
                Sync.PREPARE,
                cmd_id,
                seq_no
            ),
            response=True
        )
        # Since we're using write w/ response, use post-write time as t4
        t4 = now_ms()
        # PTP/2PC from follower to leader
        t2_e, sync_resp_id, resp_seq_no, _ = struct.unpack(
            RESP_LAYOUT,
            await self.buckler.read_gatt_char(self.resp_ch)
        )
        # Peripheral assumes t2 = t3 (we don't need that kind of granularity)
        rtt = t4 - t1
        err = t2_e - t1 + rtt // 2
        print(f"# {i} estimated RTT: {rtt}; e: {err}")
        if resp_seq_no != seq_no:
            print(f"# {i} unexpected sequence number: got {resp_seq_no}, exp {seq_no}")
            return None
        if sync_resp_id == Sync.VOTE_COMMIT:
            print(f"# {i} voted commit [t2+e={t2_e}, t4={t4}]")
            return err
        elif sync_resp_id == Sync.VOTE_ABORT:
            print(f"# {i} voted abort")
            return None
        else:
            print(f"# {i} voted invalid ({sync_resp_id})")
            return None

    async def wait_for_ack(self, seq_no):
        _, sync_resp_id, resp_seq_no, _ = struct.unpack(
            RESP_LAYOUT,
            await self.buckler.read_gatt_char(self.resp_ch)
        )
        if resp_seq_no != seq_no:
            print(f"# {self.id} unexpected sequence number: got {resp_seq_no}, exp {seq_no}")
            return False
        return sync_resp_id == Sync.ACK

    async def commit(self, seq_no, err):
        print(f"# {self.id} sending 2PC commit [e={err}]")
        await self.buckler.write_gatt_char(
            self.req_ch,
            # Commit:
            # - _: u32
            # - e: i32
            # - 2PC command: u8
            # - _: u8
            # - seq_no: u8
            struct.pack(REQ_COMMIT_LAYOUT, 0, err, Sync.COMMIT, 0, seq_no),
            response=True
        )
        return await self.wait_for_ack(seq_no)

    async def abort(self, seq_no):
        print(f"# {self.id} sending 2PC abort")
        await self.buckler.write_gatt_char(
            self.req_ch,
            struct.pack(REQ_PREPARE_LAYOUT, 0, 0, Sync.ABORT, 0),
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

        seq_no = random.randint(0, 256)
        while True:
            seq_no += 1
            seq_no %= 256 # Ensure doesn't overflow byte
            cmd = input(f"ddd seq_no={seq_no}> ").strip()
            if cmd == "q":
                print("quitting")
                break
            if cmd in CMD_LUT:
                print(f"# Beginning 2PC sequence {seq_no}")
                cmd_id = CMD_LUT[cmd]
                t1 = now_ms()
                # [p1, p2] = await asyncio.gather(
                #     ch_0.prepare(t1, cmd_id),
                #     ch_1.prepare(t1, cmd_id),
                # )
                err0 = await ch_0.prepare(seq_no, t1, cmd_id)
                if not err0:
                    print("# Aborting 2PC")
                    await ch_0.abort(seq_no)
                    continue
                c0 = await ch_0.commit(seq_no, err0)
                if not c0:
                    print("# Didn't get 2PC ACK (nothing left to do)")
            else:
                print(f"invalid command: {cmd}")
    finally:
        await asyncio.gather(buckler_0.disconnect(), buckler_1.disconnect())

loop = asyncio.get_event_loop()
loop.run_until_complete(run())
