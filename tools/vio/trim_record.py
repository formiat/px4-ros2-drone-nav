#!/usr/bin/env python3
"""trim_record.py DIR: cut the preallocated frame files of a record to their used length."""
import os, sys
d = sys.argv[1]; end = {'left': 0, 'right': 0}
for line in open(os.path.join(d, 'frames.csv')).read().strip().split('\n')[1:]:
    side, _, _, w, h, offset = line.split(','); end[side] = max(end[side], int(offset) + -(-int(w) * int(h) // 4096) * 4096)
for side, size in end.items(): os.truncate(os.path.join(d, side + '.bin'), size)
print(end)
