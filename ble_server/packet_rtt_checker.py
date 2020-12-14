#!/usr/bin/env python3
"""
Runs a sequence of a few commands to calculate round-trip-times, then plots them.
Just run this file to collect samples; change SAMPLE_COUNT as needed.

Mostly copy/pasted from ble_server.
"""

import asyncio
import struct
import random
import time
import matplotlib.pyplot as plt
from bleak import BleakClient, BleakScanner

SAMPLE_COUNT = 39

prepare_delay_ms = 3000

DDD_SERVICE_UUID     = "32e61089-2b22-4db5-a914-43ce41986c70"
DDD_REQ_CHAR_UUID    = "32e6108b-2b22-4db5-a914-43ce41986c70"
DDD_RESP_CHAR_UUID   = "32e6108c-2b22-4db5-a914-43ce41986c70"
DDD_NOSYNC_CHAR_UUID = "32e6108d-2b22-4db5-a914-43ce41986c70"

CONN_TIMEOUT = 20.0 # seconds

# < means little endian
# L means unsigned long (4B), l means signed long, B means unsigned byte
REQ_PREPARE_LAYOUT = "<LBBB"
REQ_COMMIT_LAYOUT = "<lBBB"
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
    "go": 8,
    "stop": 9,
}

start_ms = int(time.time() * 1000)

def now_ms():
    # Can't use process_time since we're probably sleeping a lot with async
    return int(time.time() * 1000) - start_ms

class Channels:
    def __init__(self, i, buckler, addr, chs):
        self.id = i
        self.buckler = buckler
        self.addr = addr
        self.req_ch = chs[0] 
        self.resp_ch = chs[1]
        self.nosync_ch = chs[2]
        self.recorded_rtts = []

    async def do_nosync(self, cmd_id):
        return await self.buckler.write_gatt_char(
            self.nosync_ch,
            struct.pack("B", cmd_id),
            response=False
        )

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
            # - target time [ms]: u32
            # - 2PC command: u8
            # - robot command: u8
            # - seq_no: u8
            struct.pack(
                REQ_PREPARE_LAYOUT,
                t1 + prepare_delay_ms,
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
        self.recorded_rtts.append(rtt)
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
            struct.pack(REQ_COMMIT_LAYOUT, err, Sync.COMMIT, 0, seq_no),
            response=True
        )
        return await self.wait_for_ack(seq_no)

    async def abort(self, seq_no):
        print(f"# {self.id} sending 2PC abort")
        await self.buckler.write_gatt_char(
            self.req_ch,
            struct.pack(REQ_PREPARE_LAYOUT, 0, Sync.ABORT, 0),
            response=False
        )
        return await self.wait_for_ack(seq_no)

    async def disconnect(self):
        return await self.buckler.disconnect()

    async def try_reconnect(self):
        if await self.buckler.is_connected():
            print(f"{self.id} still connected, doing nothing")
        else:
            print(f"{self.id} attempting reconnect...")
            self.buckler = BleakClient(self.addr)
            self.req_ch, self.resp_ch, self.nosync_ch = await get_buckler_ch(self.buckler)
            print(f"{self.id} reconnected")
            return

async def get_buckler_ch(buckler):
    await buckler.connect(timeout=CONN_TIMEOUT)
    # Get service
    svs = await buckler.get_services()
    sv = svs.get_service(DDD_SERVICE_UUID)
    # Get characteristic
    return (
        sv.get_characteristic(DDD_REQ_CHAR_UUID).handle,
        sv.get_characteristic(DDD_RESP_CHAR_UUID).handle,
        sv.get_characteristic(DDD_NOSYNC_CHAR_UUID).handle,
    )

def plot(rtts_0, rtts_1):
    plt.plot(rtts_0, "b.", ms=8, label="Robot 0")
    plt.plot(rtts_1, "gx", ms=8, label="Robot 1")
    plt.axis([0, SAMPLE_COUNT - 1, 0, max(max(rtts_0), max(rtts_1)) * 1.1])
    # # Plot average lines
    # xs = np.arange(0, SAMPLE_COUNT, 0.2)
    # mean_0 = sum(rtts_0) / len(rtts_0)
    # mean_1 = sum(rtts_1) / len(rtts_1)
    # plt.plot(xs, [mean_0 for _ in xs], "bs", ms=1, label="Robot 0 Mean")
    # plt.plot(xs, [mean_1 for _ in xs], "gs", ms=1, label="Robot 1 Mean")
    plt.ylabel("Approximate RTT (ms)")
    plt.xlabel("Sample Number")
    plt.grid(True)
    plt.legend()
    plt.show()

# https://bleak.readthedocs.io/en/latest/usage.html
async def run():
    print("searching for DDDs...")
    buckler_0 = None
    buckler_1 = None
    addr_0 = None
    addr_1 = None
    while buckler_0 is None or buckler_1 is None:
        devices = await BleakScanner.discover()
        for d in devices:
            # print(f"device {d} at address {d.address}")
            if "EE149" in d.name:
                print(f"discovered device {d}")
                if buckler_0 is None and "(0)" in d.name:
                    buckler_0 = BleakClient(d.address)
                    addr_0 = d.address
                    print(f"discovered DDD 0 @ addr {d.address}")
                    if buckler_1 is not None:
                        break
                if buckler_1 is None and "(1)" in d.name:
                    buckler_1 = BleakClient(d.address)
                    addr_1 = d.address
                    print(f"discovered DDD 1 @ addr {d.address}")
                    if buckler_0 is not None:
                        break
        if buckler_0 is None or buckler_1 is None:
            print("retrying discovery...")

    ch_0 = None
    ch_1 = None
    try:
        print("DDDs found, verifying characteristics...")
        channel_tuples = await asyncio.gather(
            get_buckler_ch(buckler_0),
            get_buckler_ch(buckler_1)
        )
        ch_0 = Channels(0, buckler_0, addr_0, channel_tuples[0])
        ch_1 = Channels(1, buckler_1, addr_1, channel_tuples[1])
        print("Both DDDs ready!")
        print("Sending go, then waiting 2s")
        seq_no = random.randint(0, 256)
        # Send go command
        await asyncio.gather(
            ch_0.do_nosync(8),
            ch_1.do_nosync(8)
        )
        time.sleep(2)
        for i in range(SAMPLE_COUNT):
            print(f"BEGINNING SAMPLE {i}")
            # Toggle between forward and back
            cmd_id = i % 2 + 5
            seq_no += 1
            seq_no %= 256 # Ensure doesn't overflow byte
            print(f"# Beginning 2PC sequence {seq_no}")
            t1 = now_ms()
            err0, err1 = await asyncio.gather(
                ch_0.prepare(seq_no, t1, cmd_id),
                ch_1.prepare(seq_no, t1, cmd_id)
            )
            if not err0 or not err1:
                print("# Aborting 2PC")
                await asyncio.gather(
                    ch_0.abort(seq_no),
                    ch_1.abort(seq_no)
                )
                continue
            c0, c1 = await asyncio.gather(
                ch_0.commit(seq_no, err0),
                ch_1.commit(seq_no, err1)
            )
            if not c0 or not c1:
                print("# Missed a 2PC ACK (nothing left to do)")
            time.sleep(0.8)
        # Send zero command
        await asyncio.gather(
            ch_0.do_nosync(7),
            ch_1.do_nosync(7)
        )
        rtts_0 = ch_0.recorded_rtts
        rtts_1 = ch_1.recorded_rtts
        print(f"DDD 0 RTTs: {rtts_0}")
        print(f"DDD 1 RTTs: {rtts_1}")
        plot(rtts_0, rtts_1)
    finally:
        await asyncio.gather(ch_0.disconnect(), ch_1.disconnect())

# loop = asyncio.get_event_loop()
# loop.run_until_complete(run())

# I saved every data point in a file for reasons
def read_points():
    with open("times_0.txt") as f0:
        points_0 = [int(l.strip()) for l in f0.readlines()]
        with open("times_1.txt") as f1:
            points_1 = [int(l.strip()) for l in f1.readlines()]
            plot(points_0, points_1)

read_points()

