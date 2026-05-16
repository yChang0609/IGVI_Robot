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

The host agent is intentionally the only component that touches the Docker Engine API. The UI talks to it through localhost HTTP.
