from __future__ import annotations

import argparse


def main() -> None:
    parser = argparse.ArgumentParser(description="IGVI Robot app launcher")
    parser.add_argument(
        "target",
        nargs="?",
        default="ui",
        choices=("ui", "host"),
        help="Component to start",
    )
    args = parser.parse_args()

    if args.target == "host":
        from igvi_host.server import main as host_main

        host_main()
        return

    from igvi_ui.app import main as ui_main

    ui_main()


if __name__ == "__main__":
    main()
