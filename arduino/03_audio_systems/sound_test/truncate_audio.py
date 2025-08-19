#!/usr/bin/env python3

# Read the first 20000 bytes from whileDancing.raw and create a medium-sized version
with open('c:/Users/cityz/IllI/play/soulShine/whileDancing.raw', 'rb') as f:
    data = f.read(20000)

with open('c:/Users/cityz/IllI/play/soulShine/whileDancing_medium.raw', 'wb') as out:
    out.write(data)

print(f'Created whileDancing_medium.raw with {len(data)} bytes')