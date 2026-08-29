#!/usr/bin/env python3
"""Select a reproducible, class-balanced 100-image review set from three YOLO sets."""

from __future__ import annotations

import csv
import hashlib
import os
import re
import shutil
from collections import Counter
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


SOURCES = [
    ("20260728", Path("/home/parkjinwoo/Downloads/20260728 1")),
    ("halla0727", Path("/home/parkjinwoo/Downloads/halla0727(Segmentation) 1/halla0727(Detection)(2)")),
    ("hanla0721", Path("/home/parkjinwoo/Downloads/hanla_0721")),
]
OUT = Path("/home/parkjinwoo/urrc_hanla/test_images_combined3_100")
NAMES = ["road", "W_line", "Y_line", "R_light", "Y_light", "G_light", "Left", "etc_light", "stop", "traffic20", "C_line", "words"]
QUOTAS = {"20260728": 50, "halla0727": 30, "hanla0721": 20}
EXTS = {".jpg", ".jpeg", ".png", ".bmp"}


def frame_number(path: Path) -> int:
    nums = re.findall(r"\d+", path.stem)
    return int(nums[-1]) if nums else 0


def load_items():
    items = []
    seen_hashes = set()
    for source, root in SOURCES:
        for image in sorted((root / "images/train").iterdir()):
            if image.suffix.lower() not in EXTS:
                continue
            label = root / "labels/train" / f"{image.stem}.txt"
            if not label.exists():
                continue
            digest = hashlib.sha1(image.read_bytes()).hexdigest()
            if digest in seen_hashes:
                continue
            seen_hashes.add(digest)
            classes = set()
            valid = True
            for line in label.read_text(errors="replace").splitlines():
                fields = line.split()
                if not fields:
                    continue
                try:
                    class_id = int(float(fields[0]))
                except ValueError:
                    valid = False
                    break
                if class_id < 0 or class_id >= len(NAMES) or len(fields) < 5:
                    valid = False
                    break
                classes.add(class_id)
            if valid:
                items.append({"source": source, "image": image, "label": label,
                              "classes": classes, "frame": frame_number(image), "hash": digest})
    return items


def select(items):
    frequency = Counter(c for x in items for c in x["classes"])
    selected = []
    selected_ids = set()
    selected_frames = {name: [] for name, _ in SOURCES}
    selected_classes = Counter()

    for source, _ in SOURCES:
        pool = [x for x in items if x["source"] == source]
        for _ in range(min(QUOTAS[source], len(pool))):
            candidates = [x for x in pool if id(x) not in selected_ids]
            if not candidates:
                break

            def score(x):
                # Reward rare/uncovered classes and frames far from already selected neighbors.
                class_score = sum((3.0 if selected_classes[c] == 0 else 1.0 / (1 + selected_classes[c]))
                                  / max(1, frequency[c]) ** 0.5 for c in x["classes"])
                neighbor = min((abs(x["frame"] - f) for f in selected_frames[source]), default=9999)
                spacing_score = min(neighbor, 30) / 30.0
                empty_penalty = -0.5 if not x["classes"] else 0.0
                return class_score * 12.0 + spacing_score + empty_penalty

            chosen = max(candidates, key=lambda x: (score(x), -x["frame"]))
            selected.append(chosen)
            selected_ids.add(id(chosen))
            selected_frames[source].append(chosen["frame"])
            selected_classes.update(chosen["classes"])
    return selected


def write_output(selected, total_items):
    if OUT.exists():
        shutil.rmtree(OUT)
    (OUT / "images").mkdir(parents=True)
    (OUT / "labels").mkdir()

    rows = []
    for index, item in enumerate(selected, 1):
        base = f"{index:03d}_{item['source']}_{item['image'].stem}"
        dst_image = OUT / "images" / f"{base}{item['image'].suffix.lower()}"
        dst_label = OUT / "labels" / f"{base}.txt"
        os.symlink(item["image"], dst_image)
        os.symlink(item["label"], dst_label)
        rows.append([index, item["source"], str(item["image"]), dst_image.name,
                     " ".join(NAMES[c] for c in sorted(item["classes"])), item["hash"]])

    with (OUT / "selection_report.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["index", "source", "original_image", "selected_name", "classes", "sha1"])
        writer.writerows(rows)

    (OUT / "selected_images.txt").write_text("\n".join(row[2] for row in rows) + "\n")
    yaml_names = "\n".join(f"  {i}: {name}" for i, name in enumerate(NAMES))
    (OUT / "data.yaml").write_text(f"path: {OUT}\ntrain: images\nval: images\nnames:\n{yaml_names}\n")

    source_counts = Counter(x["source"] for x in selected)
    class_counts = Counter(c for x in selected for c in x["classes"])
    report = [
        "# combined_3 모델 확인용 100장",
        "",
        "세 원본 데이터셋에서 중복 파일을 제거하고 희소 클래스와 프레임 간격을 고려해 선정했습니다.",
        "이미지와 라벨은 원본을 가리키는 심볼릭 링크이므로 원본을 수정하지 않습니다.",
        "학습에 사용된 사진이라면 독립 성능평가가 아니라 회귀/육안 확인용입니다.",
        "",
        f"- 유효 후보: {total_items}장",
        f"- 선정: {len(selected)}장",
        *[f"- {k}: {v}장" for k, v in source_counts.items()],
        "",
        "## 포함 이미지 수(한 이미지에 여러 클래스 가능)",
        "",
        *[f"- {NAMES[i]}: {class_counts[i]}장" for i in range(len(NAMES))],
    ]
    (OUT / "README.md").write_text("\n".join(report) + "\n")

    thumbs = []
    for item, row in zip(selected, rows):
        with Image.open(item["image"]) as im:
            im = im.convert("RGB")
            im.thumbnail((180, 110))
            tile = Image.new("RGB", (190, 140), "white")
            tile.paste(im, ((190 - im.width) // 2, 2))
            ImageDraw.Draw(tile).text((4, 116), f"{row[0]:03d} {item['source']} {item['image'].stem}", fill="black")
            thumbs.append(tile)
    sheet = Image.new("RGB", (1900, ((len(thumbs) + 9) // 10) * 140), "#dddddd")
    for i, thumb in enumerate(thumbs):
        sheet.paste(thumb, ((i % 10) * 190, (i // 10) * 140))
    sheet.save(OUT / "contact_sheet.jpg", quality=90)


def main():
    items = load_items()
    selected = select(items)
    write_output(selected, len(items))
    print((OUT / "README.md").read_text())


if __name__ == "__main__":
    main()
