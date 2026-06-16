import re, pathlib

fixes = {
    "camera_viewer_3.py": (
        '    print(f"[端口] {cli.port}")\n'
        '    print(f"[访问] http://192.168.5.4:{cli.port}")\n'
        '    print(f"[流]   头部: http://192.168.5.4:{cli.port}/stream/head")\n'
        '    print(f"[流]   左腕: http://192.168.5.4:{cli.port}/stream/left")\n'
        '    print(f"[流]   右腕: http://192.168.5.4:{cli.port}/stream/right")\n'
        '    print("=" * 60)',
        '    print(f"[端口] {cli.port}")\n'
        '    ip = _local_ip()\n'
        '    print(f"[访问] http://{ip}:{cli.port}")\n'
        '    print(f"[流]   头部: http://{ip}:{cli.port}/stream/head")\n'
        '    print(f"[流]   左腕: http://{ip}:{cli.port}/stream/left")\n'
        '    print(f"[流]   右腕: http://{ip}:{cli.port}/stream/right")\n'
        '    print("=" * 60)',
    ),
    "camera_viewer_1.py": (
        '    print(f"[端口] {cli.port}")\n'
        '    print(f"[访问] http://192.168.5.4:{cli.port}")\n'
        '    print(f"[流]   头部: http://192.168.5.4:{cli.port}/stream/head")\n'
        '    print("=" * 60)',
        '    print(f"[端口] {cli.port}")\n'
        '    ip = _local_ip()\n'
        '    print(f"[访问] http://{ip}:{cli.port}")\n'
        '    print(f"[流]   头部: http://{ip}:{cli.port}/stream/head")\n'
        '    print("=" * 60)',
    ),
}

base = pathlib.Path(__file__).parent
for fname, (old, new) in fixes.items():
    p = base / fname
    text = p.read_text(encoding="utf-8")
    if old in text:
        p.write_text(text.replace(old, new), encoding="utf-8")
        print(f"{fname}: OK")
    else:
        print(f"{fname}: old string not found")
