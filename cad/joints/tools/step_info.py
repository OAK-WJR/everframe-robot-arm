#!/usr/bin/env python3
"""Print the header and the part (PRODUCT) names of STEP files.

No CAD program needed; uses only the Python standard library.
Usage:  python3 tools/step_info.py mechanical/joints/D25-vertical.step [more.step ...]
"""
import re
import sys

HEADER_KEYS = ("name", "time_stamp", "originating_system")


def decode_step_string(s):
    """Decode STEP unicode escapes like \\X2\\4E0B\\X0\\ into normal text."""
    def repl(m):
        hexs = m.group(1)
        return "".join(chr(int(hexs[i:i + 4], 16)) for i in range(0, len(hexs), 4))
    return re.sub(r"\\X2\\([0-9A-Fa-f]+)\\X0\\", repl, s)


def info(path):
    with open(path, "r", encoding="latin-1") as f:
        text = f.read()
    head = text[: text.find("ENDSEC;")]
    print(f"== {path}")
    for key in HEADER_KEYS:
        m = re.search(r"/\* " + key + r" \*/\s*'([^']*)'", head)
        if m:
            print(f"  {key}: {decode_step_string(m.group(1))}")
    names = re.findall(r"=\s*PRODUCT\s*\(\s*'([^']*)'", text)
    parts = sorted({" ".join(decode_step_string(n).split()) for n in names})
    print(f"  parts ({len(parts)}):")
    for p in parts:
        print(f"    {p}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        info(p)
