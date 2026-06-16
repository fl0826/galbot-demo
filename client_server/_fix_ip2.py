import pathlib

p = pathlib.Path(__file__).parent / "camera_viewer_1.py"
text = p.read_text(encoding="utf-8")

old = "import argparse\nimport logging"
new = (
    "import argparse\n"
    "import logging\n"
    "import socket\n"
    "\n\n"
    "def _local_ip() -> str:\n"
    "    try:\n"
    "        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:\n"
    '            s.connect(("8.8.8.8", 80))\n'
    "            return s.getsockname()[0]\n"
    "    except Exception:\n"
    '        return "127.0.0.1"'
)

if old in text:
    p.write_text(text.replace(old, new, 1), encoding="utf-8")
    print("OK")
else:
    # show what's actually there
    for i, line in enumerate(text.splitlines()[25:35], 26):
        print(i, repr(line))
