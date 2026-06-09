#!/usr/bin/env python3
"""
Example: call navigation_interface from another script.
"""

from navigation_interface import navigate_to_pose, get_navigation_interface


def navigate_once():
    """Single navigation call."""
    success = navigate_to_pose(
        x=3.910509705543518,
        y=-0.3748614490032196,
        z=0.0,
        qx=0.0,
        qy=-0.06110484803730875,
        qz=0.9980884966601643,
        qw=0.0,
        timeout=30.0,
    )
    return success


def navigate_multiple():
    """Multiple navigations, reuse the same connection."""
    nav = get_navigation_interface()

    targets = [
        (1.266, 1.164, 0.0, 0.0, 0.0, 0.700, 0.714),
        (2.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0),
    ]

    for i, (x, y, z, qx, qy, qz, qw) in enumerate(targets, start=1):
        print(f"Navigating to target {i}...")
        success = navigate_to_pose(
            x=x, y=y, z=z,
            qx=qx, qy=qy, qz=qz, qw=qw,
            timeout=20.0,
            nav_interface=nav,
        )
        if not success:
            print(f"Navigation to target {i} failed")
            return False
        print(f"Navigation to target {i} succeeded")

    return True


def main():
    print("=== Example 1: single navigation ===")
    if navigate_once():
        print("Navigation succeeded")
    else:
        print("Navigation failed")
        return

    # print("\n=== Example 2: multiple navigations ===")
    # if navigate_multiple():
    #     print("All navigations succeeded")
    # else:
    #     print("Some navigation failed")


if __name__ == "__main__":
    main()
