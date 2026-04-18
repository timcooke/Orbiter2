#!/usr/bin/env python3
"""
Generate a procedural equirectangular star-field Radiance HDR image.

Output: app/src/main/assets/environments/stars.hdr  (1024 x 512 px)

The file uses the standard Radiance HDR (RGBE) format.  Filament's HDRLoader
can read this directly via EnvironmentLoader.createHDREnvironment().
"""

import math
import os
import random
import struct
import sys


# ──────────────────────────────────────────────────────────────────
# RGBE encoding helpers
# ──────────────────────────────────────────────────────────────────

def float_to_rgbe(r: float, g: float, b: float) -> bytes:
    """Pack floating-point RGB into 4-byte Radiance RGBE."""
    max_c = max(r, g, b)
    if max_c < 1e-32:
        return b'\x00\x00\x00\x00'
    mantissa, exp = math.frexp(max_c)   # value = mantissa * 2**exp, mantissa in [0.5, 1.0)
    scale = mantissa * 256.0 / max_c
    return bytes([
        min(255, int(r * scale)),
        min(255, int(g * scale)),
        min(255, int(b * scale)),
        exp + 128,
    ])


# ──────────────────────────────────────────────────────────────────
# RLE scanline encoder (new Radiance format)
# ──────────────────────────────────────────────────────────────────

def rle_encode_channel(values: list) -> bytes:
    """
    Encode a single component channel using Radiance scanline RLE.
    count < 128  → literal run: [count, v0, v1, …]
    count >= 128 → repeat run : [count, value]  where count = 128 + repetitions
    """
    out = bytearray()
    n = len(values)
    i = 0
    while i < n:
        # Look for a repeat run (≥ 2 identical values)
        run_start = i
        run_val = values[i]
        run_len = 1
        while run_len < 127 and i + run_len < n and values[i + run_len] == run_val:
            run_len += 1

        if run_len >= 2:
            out.append(128 + run_len)
            out.append(run_val)
            i += run_len
        else:
            # Collect a literal run (up to 127 values that don't form a repeat).
            # Radiance literal count byte must be < 128 (0x80 = repeat marker).
            lit = [values[i]]
            i += 1
            while len(lit) < 127 and i < n:
                # Stop before a run of 2+ identical values
                if i + 1 < n and values[i] == values[i + 1]:
                    break
                lit.append(values[i])
                i += 1
            out.append(len(lit))
            out.extend(lit)
    return bytes(out)


def encode_scanline(row_rgbe: list) -> bytes:
    """
    Encode one scanline in the new Radiance RLE format.
    row_rgbe: list of (r_byte, g_byte, b_byte, e_byte) tuples.
    """
    W = len(row_rgbe)
    # Scanline header: [2, 2, W>>8, W&0xFF]
    header = bytes([2, 2, (W >> 8) & 0xFF, W & 0xFF])
    # Split into 4 separate channels
    channels = [[pix[ch] for pix in row_rgbe] for ch in range(4)]
    body = b''.join(rle_encode_channel(ch) for ch in channels)
    return header + body


# ──────────────────────────────────────────────────────────────────
# Star-field generation
# ──────────────────────────────────────────────────────────────────

def gaussian_blur(pixels, width, height, cx, cy, sigma, r, g, b):
    """Add a Gaussian blob (a soft star) centred at (cx, cy)."""
    radius = max(1, int(sigma * 3))
    for dy in range(-radius, radius + 1):
        for dx in range(-radius, radius + 1):
            nx, ny = (cx + dx) % width, cy + dy
            if 0 <= ny < height:
                weight = math.exp(-(dx*dx + dy*dy) / (2 * sigma * sigma))
                idx = ny * width + nx
                pixels[idx] = [
                    min(20.0, pixels[idx][0] + r * weight),
                    min(20.0, pixels[idx][1] + g * weight),
                    min(20.0, pixels[idx][2] + b * weight),
                ]


def generate_starfield(width: int, height: int, seed: int = 42) -> list:
    """
    Return a flat list of [r, g, b] float pixels (length = width * height).
    Coordinate (0,0) is top-left; x increases right, y increases down.
    """
    rng = random.Random(seed)

    # Background: deep space black with a very faint blue tint
    pixels = [[0.001, 0.001, 0.003] for _ in range(width * height)]

    # ── Faint background star field ──────────────────────────────
    num_faint = 3500
    for _ in range(num_faint):
        # Uniform distribution over the sphere projected equirectangularly
        # (importance-sample cosine latitude so polar caps aren't over-dense)
        u = rng.random()   # longitude  0..1
        v = rng.random()   # sin(lat)  -1..1  → use acos for uniform sphere
        lon = u * 2 * math.pi
        lat = math.asin(v * 2 - 1)           # uniform on sphere
        px = int((lon / (2 * math.pi)) * width)  % width
        py = int(((math.pi / 2 - lat) / math.pi) * height)
        py = max(0, min(height - 1, py))

        brightness = rng.uniform(0.04, 0.35)
        # Slight colour variation: most are white-ish, some blue-white, a few warm
        tint = rng.choice([
            (1.0, 1.0, 1.0),           # pure white
            (0.85, 0.92, 1.0),         # blue-white
            (1.0, 0.95, 0.80),         # warm yellow-white
        ])
        idx = py * width + px
        pixels[idx] = [
            min(20.0, pixels[idx][0] + brightness * tint[0]),
            min(20.0, pixels[idx][1] + brightness * tint[1]),
            min(20.0, pixels[idx][2] + brightness * tint[2]),
        ]

    # ── Bright stars (with soft glow) ────────────────────────────
    num_bright = 120
    for _ in range(num_bright):
        u = rng.random()
        v = rng.random()
        lon = u * 2 * math.pi
        lat = math.asin(v * 2 - 1)
        px = int((lon / (2 * math.pi)) * width) % width
        py = int(((math.pi / 2 - lat) / math.pi) * height)
        py = max(0, min(height - 1, py))

        brightness = rng.uniform(1.5, 8.0)
        sigma = rng.uniform(0.10, 0.30)   # tighter glow → crisp point + tiny halo
        tint = rng.choice([
            (1.0, 1.0, 1.0),
            (0.8, 0.9, 1.0),   # blue-white (hot star)
            (1.0, 0.85, 0.6),  # orange giant
        ])
        gaussian_blur(pixels, width, height, px, py, sigma,
                      brightness * tint[0],
                      brightness * tint[1],
                      brightness * tint[2])

    return pixels


# ──────────────────────────────────────────────────────────────────
# Main
# ──────────────────────────────────────────────────────────────────

def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else \
        os.path.join(os.path.dirname(__file__),
                     '..', 'app', 'src', 'main', 'assets',
                     'environments', 'stars.hdr')
    out_path = os.path.normpath(out_path)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    WIDTH, HEIGHT = 1024, 512
    print(f"Generating {WIDTH}x{HEIGHT} equirectangular star field...")
    pixels = generate_starfield(WIDTH, HEIGHT)

    print(f"Encoding RGBE + writing {out_path} ...")
    with open(out_path, 'wb') as f:
        # Radiance HDR header
        header = (
            b'#?RADIANCE\n'
            b'FORMAT=32-bit_rle_rgbe\n'
            b'EXPOSURE=1.0\n'
            b'\n'
            + f'-Y {HEIGHT} +X {WIDTH}\n'.encode()
        )
        f.write(header)

        # Encode scanlines
        for y in range(HEIGHT):
            row = pixels[y * WIDTH:(y + 1) * WIDTH]
            row_rgbe = [
                tuple(float_to_rgbe(p[0], p[1], p[2]))
                for p in row
            ]
            f.write(encode_scanline(row_rgbe))

    size_kb = os.path.getsize(out_path) / 1024
    print(f"Done. File size: {size_kb:.1f} KB")


if __name__ == '__main__':
    main()
