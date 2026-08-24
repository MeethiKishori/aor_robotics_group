#!/usr/bin/env python3
"""Downscale a Roboflow YOLO Darknet export so darknet isn't I/O bound.

The Roboflow version was generated without a Resize preprocessing step, so the
images are full-resolution camera originals (4000x3000, ~5.4 MB each). Darknet
resizes every image to the network size on load anyway, so decoding 12 MP JPEGs
each epoch is pure waste -- it starves the GPU.

Scaling is **uniform and aspect-preserving**, which is what makes this safe:
YOLO labels are normalized (cx = x_px / img_w), so scaling both the coordinate
and the dimension by the same factor leaves the value unchanged --
(k*x)/(k*w) == x/w. The .txt files are therefore copied byte-for-byte.

Letterboxing to a square would NOT have this property (padding shifts the
origin), so we deliberately don't; darknet letterboxes to the network's
square input itself at load time.

Usage:
    python3 resize_dataset.py <src-export> <dst-export> [max_side]
"""

import os
import shutil
import sys
from concurrent.futures import ProcessPoolExecutor

from PIL import Image

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")
SPLITS = ("train", "valid", "test")


def resize_one(job):
    src, dst, max_side = job
    with Image.open(src) as im:
        w, h = im.size
        if max(w, h) <= max_side:
            shutil.copy2(src, dst)
            return 0

        scale = max_side / max(w, h)
        new_size = (max(1, round(w * scale)), max(1, round(h * scale)))

        # draft() lets libjpeg decode straight to ~the target scale, which is
        # several times faster than decoding 12 MP and then downsampling.
        if im.format == "JPEG":
            im.draft("RGB", new_size)

        im = im.convert("RGB").resize(new_size, Image.LANCZOS)
        im.save(dst, quality=90, optimize=True)
    return 1


def main():
    if len(sys.argv) not in (3, 4):
        raise SystemExit(__doc__.strip().splitlines()[-1].strip())

    src_root = os.path.abspath(sys.argv[1])
    dst_root = os.path.abspath(sys.argv[2])
    max_side = int(sys.argv[3]) if len(sys.argv) == 4 else 640

    if not os.path.isdir(src_root):
        raise SystemExit(f"source not found: {src_root}")
    if os.path.exists(dst_root):
        raise SystemExit(f"destination already exists: {dst_root}")

    jobs = []
    for split in SPLITS:
        src_dir = os.path.join(src_root, split)
        if not os.path.isdir(src_dir):
            continue
        dst_dir = os.path.join(dst_root, split)
        os.makedirs(dst_dir, exist_ok=True)

        for name in sorted(os.listdir(src_dir)):
            src = os.path.join(src_dir, name)
            dst = os.path.join(dst_dir, name)
            if name.lower().endswith(IMG_EXT):
                jobs.append((src, dst, max_side))
            elif os.path.isfile(src):
                shutil.copy2(src, dst)   # .txt labels, _darknet.labels: verbatim

    # Top-level READMEs etc.
    for name in os.listdir(src_root):
        path = os.path.join(src_root, name)
        if os.path.isfile(path):
            shutil.copy2(path, os.path.join(dst_root, name))

    print(f"resizing {len(jobs)} images to max side {max_side}...")
    with ProcessPoolExecutor() as pool:
        resized = sum(pool.map(resize_one, jobs, chunksize=8))

    print(f"done: {resized} resized, {len(jobs) - resized} already small enough")
    for split in SPLITS:
        d = os.path.join(dst_root, split)
        if os.path.isdir(d):
            n_img = sum(1 for f in os.listdir(d) if f.lower().endswith(IMG_EXT))
            n_txt = sum(1 for f in os.listdir(d) if f.endswith(".txt"))
            print(f"  {split}: {n_img} images, {n_txt} txt")


if __name__ == "__main__":
    main()
