import os
import sys


def main() -> int:
    # Allow running via:
    #   python3 libs/galbot_vla_real/tool/vla_profile_viewer.py ...
    # by making `tool.*` importable.
    this_dir = os.path.dirname(os.path.abspath(__file__))
    repo_pkg_root = os.path.abspath(os.path.join(this_dir, ".."))
    if repo_pkg_root not in sys.path:
        sys.path.insert(0, repo_pkg_root)

    from tool.vla_profile_viewer_app.cli import main as _main

    return int(_main())


if __name__ == "__main__":
    raise SystemExit(main())

