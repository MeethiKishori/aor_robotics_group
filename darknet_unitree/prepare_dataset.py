#!/usr/bin/env python3
"""Generate the index files darknet needs from a Roboflow "YOLO Darknet" export.

No format conversion happens here and none is needed -- the export's .txt files
are already darknet's native label format (`class_id cx cy w h`, normalized),
sitting beside their images exactly where darknet looks for them.

What darknet does *not* get from Roboflow, and what this writes:

    train.txt / valid.txt   absolute image paths, one per line
    obj.names               class names, one per line (order = class id)
    obj.data                points darknet at all of the above + backup/

Usage:
    python3 prepare_dataset.py <path/to/roboflow-darknet-export>
"""

import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")

# Roboflow names the validation split "valid"; accept "val" as well.
SPLITS = {"train": ("train",), "valid": ("valid", "val")}


def find_split_dir(dataset, candidates):
    for name in candidates:
        path = os.path.join(dataset, name)
        if os.path.isdir(path):
            return path
    return None


def list_split(split_dir):
    """Image paths that have a label beside them, plus the unlabelled count."""
    paths, missing = [], 0
    for name in sorted(os.listdir(split_dir)):
        if not name.lower().endswith(IMG_EXT):
            continue
        label = os.path.join(split_dir, os.path.splitext(name)[0] + ".txt")
        if not os.path.exists(label):
            missing += 1
            continue
        paths.append(os.path.join(split_dir, name))
    return paths, missing


def read_class_names(dataset):
    for candidate in (
        os.path.join(dataset, "train", "_darknet.labels"),
        os.path.join(dataset, "_darknet.labels"),
    ):
        if os.path.exists(candidate):
            with open(candidate) as f:
                names = [line.strip() for line in f if line.strip()]
            if names:
                return names
    raise SystemExit(
        f"No _darknet.labels found in {dataset}.\n"
        "Re-export from Roboflow with format 'YOLO Darknet'."
    )


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__.strip().splitlines()[-1].strip())

    dataset = os.path.abspath(sys.argv[1])
    if not os.path.isdir(dataset):
        raise SystemExit(f"dataset not found: {dataset}")

    classes = read_class_names(dataset)
    print(f"dataset: {dataset}")
    print(f"classes ({len(classes)}): {', '.join(classes)}")

    os.makedirs(os.path.join(HERE, "backup"), exist_ok=True)

    for out_name, candidates in SPLITS.items():
        split_dir = find_split_dir(dataset, candidates)
        if split_dir is None:
            raise SystemExit(f"no '{'/'.join(candidates)}' split in {dataset}")

        paths, missing = list_split(split_dir)
        if not paths:
            raise SystemExit(f"no labelled images in {split_dir}")

        with open(os.path.join(HERE, f"{out_name}.txt"), "w") as f:
            f.write("\n".join(paths) + "\n")
        note = f" ({missing} unlabelled, skipped)" if missing else ""
        print(f"{out_name}.txt: {len(paths)} images{note}")

    with open(os.path.join(HERE, "obj.names"), "w") as f:
        f.write("\n".join(classes) + "\n")

    with open(os.path.join(HERE, "obj.data"), "w") as f:
        f.write(
            f"classes = {len(classes)}\n"
            f"train = {os.path.join(HERE, 'train.txt')}\n"
            f"valid = {os.path.join(HERE, 'valid.txt')}\n"
            f"names = {os.path.join(HERE, 'obj.names')}\n"
            f"backup = {os.path.join(HERE, 'backup')}\n"
        )

    print("wrote obj.names, obj.data")
    print(f"\nnext: ./prepare_cfg.sh {len(classes)}")


if __name__ == "__main__":
    main()
