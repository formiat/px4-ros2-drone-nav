#!/usr/bin/env python3
"""ntu_magnetometer_extract.py BAG OUT_DIR: the VN-100's magnetometer and temperature out of the NTU VIRAL ROS1 bag
(v2.0, its index read for the chunks), written as magnetometer.csv (stamp_ns, x, y, z in tesla) and
temperature.csv (stamp_ns, degrees C). Logs the start, the chunk total and every 500 chunks."""
import bz2
import struct
import sys
import time
from pathlib import Path

WANTED = ("/imu/magnetic_field", "/imu/temperature")


def fields(buf):
    out, i = {}, 0
    while i < len(buf):
        n = struct.unpack_from("<I", buf, i)[0]
        i += 4
        k, _, v = buf[i:i + n].partition(b"=")
        i += n
        out[k.decode()] = v
    return out


def record(f):
    hl = struct.unpack("<I", f.read(4))[0]
    h = fields(f.read(hl))
    dl = struct.unpack("<I", f.read(4))[0]
    return h, f.read(dl)


def records(buf):
    i = 0
    while i < len(buf):
        hl = struct.unpack_from("<I", buf, i)[0]
        h = fields(buf[i + 4:i + 4 + hl])
        i += 4 + hl
        dl = struct.unpack_from("<I", buf, i)[0]
        yield h, buf[i + 4:i + 4 + dl]
        i += 4 + dl


def header_end(data):
    # std_msgs/Header: seq, stamp sec, stamp nsec, frame_id (length-prefixed)
    _, sec, nsec, n = struct.unpack_from("<IIII", data, 0)
    return sec * 1_000_000_000 + nsec, 16 + n


def main():
    bag, out = Path(sys.argv[1]), Path(sys.argv[2])
    out.mkdir(parents=True, exist_ok=True)
    with bag.open("rb") as f:
        assert f.read(13) == b"#ROSBAG V2.0\n"
        h, _ = record(f)
        index_pos = struct.unpack("<Q", h["index_pos"])[0]
        conn_count = struct.unpack("<I", h["conn_count"])[0]
        chunk_count = struct.unpack("<I", h["chunk_count"])[0]
        f.seek(index_pos)
        topics = {}
        for _ in range(conn_count):
            h, _ = record(f)
            topics[struct.unpack("<I", h["conn"])[0]] = h["topic"].decode()
        wanted = {c: t for c, t in topics.items() if t in WANTED}
        positions = []
        for _ in range(chunk_count):
            h, d = record(f)
            n = struct.unpack("<I", h["count"])[0]
            conns = {struct.unpack_from("<II", d, 8 * k)[0] for k in range(n)}
            if conns & set(wanted):
                positions.append(struct.unpack("<Q", h["chunk_pos"])[0])
        start = time.time()
        print(f"EXTRACT start {time.strftime('%H:%M:%S')} chunks {len(positions)} of {chunk_count} hold {sorted(wanted.values())}",
              flush=True)
        mag, temp = [], []
        for done, position in enumerate(positions, 1):
            f.seek(position)
            h, d = record(f)
            compression = h.get("compression", b"none").decode()
            if compression == "bz2":
                d = bz2.decompress(d)
            elif compression != "none":
                raise RuntimeError(f"chunk compression {compression} not read here")
            for rh, rd in records(d):
                if rh.get("op") != b"\x02":
                    continue
                conn = struct.unpack("<I", rh["conn"])[0]
                topic = wanted.get(conn)
                if topic is None:
                    continue
                stamp, i = header_end(rd)
                if topic == "/imu/magnetic_field":
                    mag.append((stamp,) + struct.unpack_from("<3d", rd, i))
                else:
                    temp.append((stamp, struct.unpack_from("<d", rd, i)[0]))
            if done % 500 == 0 or done == len(positions):
                rate = done / max(time.time() - start, 1e-9)
                print(f"EXTRACT {done}/{len(positions)} chunks, {len(mag)} fields, {rate:.0f} chunks/s, "
                      f"{(len(positions) - done) / rate:.0f} s left", flush=True)
    mag.sort()
    temp.sort()
    with (out / "magnetometer.csv").open("w") as stream:
        stream.write("stamp_ns,x_t,y_t,z_t\n")
        for s, x, y, z in mag:
            stream.write(f"{s},{x:.9g},{y:.9g},{z:.9g}\n")
    with (out / "temperature.csv").open("w") as stream:
        stream.write("stamp_ns,celsius\n")
        for s, c in temp:
            stream.write(f"{s},{c:.4f}\n")
    print(f"EXTRACT end {time.strftime('%H:%M:%S')} fields {len(mag)} temperatures {len(temp)}", flush=True)


if __name__ == "__main__":
    main()
