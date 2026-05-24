# IGVI apps

Local Python tools for the IGVI robot stack:

- `igvi-host`: localhost FastAPI host agent for Docker/Compose, IGVI bridge control, and UI bridge health.
- `igvi-ui`: PySide6 desktop control center.

Run:

```bash
cd apps
uv sync
uv run igvi-host
uv run igvi-ui
```

Linux Qt prerequisites:

```bash
sudo apt update && sudo apt install -y libxcb-cursor0
```

Run `igvi-ui` from a graphical desktop session. If you are using SSH, enable X11 forwarding or use a remote desktop session so `$DISPLAY` or `$WAYLAND_DISPLAY` is available.

The host agent is intentionally the only component that touches the Docker Engine API. The UI talks to it through localhost HTTP.
