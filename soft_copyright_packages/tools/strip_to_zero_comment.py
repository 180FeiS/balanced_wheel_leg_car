#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Strip empty lines and all C comments; remove copyright/author lines; rebuild MERGED + CPCC splits."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LINES_PER_PAGE = 50
FRONT_PAGES = 30
BACK_PAGES = 30
FRONT_LINES = LINES_PER_PAGE * FRONT_PAGES
BACK_LINES = LINES_PER_PAGE * BACK_PAGES

COPYRIGHT_PATTERNS = re.compile(
    r"(?i)(copyright|seekfree|opensourec|逐飞|成都逐飞|"
    r"created\s+on|@author|@file|author\s*:|"
    r"modified\s+record|first\s+version|pudding|bom\b|monst\b|"
    r"gnu\s+general\s+public\s+license|gpl3|shop\s+link|"
    r"公司名称|文件名称|开发环境|适用平台|修改记录|版本信息)",
)

SOURCE_SUFFIXES = {".c", ".h", ".txt"}
SKIP_NAMES = {
    "MERGED_SOURCE_ORDER.txt",
    "CPCC_FRONT_30.txt",
    "CPCC_BACK_30.txt",
    "CPCC_FULL.txt",
    "FILE_ORDER.txt",
}


def strip_c_comments(text: str) -> str:
    out: list[str] = []
    i = 0
    n = len(text)
    state = "code"
    while i < n:
        c = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if state == "code":
            if c == '"':
                out.append(c)
                state = "str"
                i += 1
            elif c == "'":
                out.append(c)
                state = "char"
                i += 1
            elif c == "/" and nxt == "/":
                state = "line_comment"
                i += 2
            elif c == "/" and nxt == "*":
                state = "block_comment"
                i += 2
            else:
                out.append(c)
                i += 1
        elif state == "str":
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
            elif c == '"':
                state = "code"
                i += 1
            else:
                i += 1
        elif state == "char":
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
            elif c == "'":
                state = "code"
                i += 1
            else:
                i += 1
        elif state == "line_comment":
            if c == "\n":
                out.append("\n")
                state = "code"
            i += 1
        elif state == "block_comment":
            if c == "*" and nxt == "/":
                state = "code"
                i += 2
            else:
                if c == "\n":
                    out.append("\n")
                i += 1
        else:
            out.append(c)
            i += 1
    return "".join(out)


def clean_line(line: str) -> str | None:
    s = line.rstrip()
    if not s.strip():
        return None
    if COPYRIGHT_PATTERNS.search(s):
        return None
    stripped = s.strip()
    if stripped.startswith("#") and not stripped.startswith("#include") and not stripped.startswith("#define"):
        if stripped.startswith("#ifndef") or stripped.startswith("#endif") or stripped.startswith("#if"):
            pass
        elif stripped.startswith("#else") or stripped.startswith("#elif"):
            pass
        elif stripped.startswith("#pragma"):
            pass
        else:
            return None
    return s.rstrip()


def process_file(path: Path) -> tuple[int, int]:
    raw = path.read_text(encoding="utf-8", errors="replace")
    stripped = strip_c_comments(raw)
    lines_out: list[str] = []
    for line in stripped.splitlines():
        cleaned = clean_line(line)
        if cleaned is not None:
            lines_out.append(cleaned)
    new_text = "\n".join(lines_out)
    if lines_out:
        new_text += "\n"
    before = len([ln for ln in raw.splitlines() if ln.strip()])
    after = len(lines_out)
    path.write_text(new_text, encoding="utf-8", newline="\n")
    return before, after


def rebuild_merged(compact_dir: Path) -> int:
    order_file = compact_dir / "FILE_ORDER.txt"
    if not order_file.exists():
        merged = compact_dir / "MERGED_SOURCE_ORDER.txt"
        if merged.exists():
            process_file(merged)
            return len(merged.read_text(encoding="utf-8").splitlines())
        return 0
    parts: list[str] = []
    for name in order_file.read_text(encoding="utf-8").splitlines():
        name = name.strip()
        if not name or name.startswith("#"):
            continue
        fp = compact_dir / name
        if not fp.exists():
            continue
        parts.append(fp.read_text(encoding="utf-8", errors="replace"))
    merged_text = "".join(parts)
    merged_path = compact_dir / "MERGED_SOURCE_ORDER.txt"
    merged_path.write_text(merged_text, encoding="utf-8", newline="\n")
    return len(merged_text.splitlines())


def write_cpcc_splits(compact_dir: Path, total: int) -> None:
    merged_path = compact_dir / "MERGED_SOURCE_ORDER.txt"
    lines = merged_path.read_text(encoding="utf-8").splitlines()
    threshold = FRONT_LINES + BACK_LINES
    if total > threshold:
        front = lines[:FRONT_LINES]
        back = lines[-BACK_LINES:]
        (compact_dir / "CPCC_FRONT_30.txt").write_text(
            "\n".join(front) + "\n", encoding="utf-8", newline="\n"
        )
        (compact_dir / "CPCC_BACK_30.txt").write_text(
            "\n".join(back) + "\n", encoding="utf-8", newline="\n"
        )
        readme = (
            f"总行数: {total}\n"
            f"前30页: CPCC_FRONT_30.txt ({len(front)} 行, 1-{FRONT_LINES})\n"
            f"后30页: CPCC_BACK_30.txt ({len(back)} 行, {total - len(back) + 1}-{total})\n"
            f"省略中间: {FRONT_LINES + 1}-{total - len(back)} 行\n"
        )
    else:
        (compact_dir / "CPCC_FULL.txt").write_text(
            merged_path.read_text(encoding="utf-8"), encoding="utf-8", newline="\n"
        )
        readme = f"总行数: {total} (<={threshold})，提交 CPCC_FULL.txt 全文即可。\n"
    (compact_dir / "CPCC_SUBMISSION_README.txt").write_text(
        readme, encoding="utf-8", newline="\n"
    )


def process_package(pkg_dir: Path) -> dict:
    compact = pkg_dir / "core_code_compact"
    if not compact.is_dir():
        return {"pkg": pkg_dir.name, "skipped": True}
    stats = []
    for fp in sorted(compact.iterdir()):
        if fp.name in SKIP_NAMES:
            continue
        if fp.suffix.lower() not in {".c", ".h"} and not fp.name.startswith(("glue__", "code__", "interfaces__")):
            continue
        if fp.suffix.lower() not in {".c", ".h"}:
            continue
        b, a = process_file(fp)
        stats.append((fp.name, b, a))
    total = rebuild_merged(compact)
    write_cpcc_splits(compact, total)
    return {"pkg": pkg_dir.name, "files": stats, "merged_lines": total}


def main() -> None:
    results = []
    for pkg in sorted(ROOT.iterdir()):
        if not pkg.is_dir() or pkg.name == "tools":
            continue
        if not (pkg / "core_code_compact").is_dir():
            continue
        results.append(process_package(pkg))
    print("=== 零注释清洗完成 ===")
    for r in results:
        if r.get("skipped"):
            continue
        print(f"\n[{r['pkg']}] MERGED {r['merged_lines']} 行")
        for name, b, a in r.get("files", []):
            print(f"  {name}: {b} -> {a} 行")


if __name__ == "__main__":
    main()
