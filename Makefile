.PHONY: help sync host ui run app

APP_DIR := apps
UV := uv

help:
	@echo "IGVI Robot app commands:"
	@echo "  make sync  - Install/sync Python dependencies"
	@echo "  make host  - Run igvi-host"
	@echo "  make ui    - Run igvi-ui"
	@echo "  make run   - Run igvi-host in the background, then open igvi-ui"
	@echo "  make app   - Alias for make run"

sync:
	cd $(APP_DIR) && $(UV) sync

host:
	cd $(APP_DIR) && $(UV) run igvi-host

ui:
	cd $(APP_DIR) && $(UV) run igvi-ui

run:
	cd $(APP_DIR) && { \
	$(UV) run igvi-host & \
	HOST_PID=$$!; \
	trap 'kill $$HOST_PID 2>/dev/null || true' INT TERM EXIT; \
	$(UV) run igvi-ui; \
	UI_STATUS=$$?; \
	kill $$HOST_PID 2>/dev/null || true; \
	wait $$HOST_PID 2>/dev/null || true; \
	exit $$UI_STATUS; \
	}

app: run
