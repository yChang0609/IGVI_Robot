.PHONY: help sync host ui run app

APP_DIR := apps
UV := uv
HOST_PORT := 8770

# `make run <ip>` connects the UI to a remote host at http://<ip>:$(HOST_PORT).
# `make run` (no IP) launches igvi-host locally and opens the UI against it.
# `make host lan` (or any non-empty arg) binds igvi-host to 0.0.0.0 so other
# machines on the LAN can connect; `make host` alone keeps the safe loopback default.
REMOTE_IP := $(filter-out run app host,$(MAKECMDGOALS))

help:
	@echo "IGVI Robot app commands:"
	@echo "  make sync          - Install/sync Python dependencies"
	@echo "  make host          - Run igvi-host (loopback only)"
	@echo "  make host lan      - Run igvi-host bound to 0.0.0.0 (exposed on LAN)"
	@echo "  make ui            - Run igvi-ui (local host)"
	@echo "  make run           - Run igvi-host in the background, then open igvi-ui"
	@echo "  make run <ip>      - Open igvi-ui pointed at a remote host (http://<ip>:$(HOST_PORT))"
	@echo "  make app           - Alias for make run"

sync:
	cd $(APP_DIR) && $(UV) sync

host:
ifeq ($(strip $(REMOTE_IP)),)
	cd $(APP_DIR) && $(UV) run igvi-host
else
	@echo "Binding igvi-host to 0.0.0.0:$(HOST_PORT) (LAN-exposed)"
	cd $(APP_DIR) && IGVI_HOST_BIND=0.0.0.0 $(UV) run igvi-host
endif

ui:
	cd $(APP_DIR) && $(UV) run igvi-ui

run:
ifeq ($(strip $(REMOTE_IP)),)
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
else
	@echo "Connecting UI to remote host at http://$(REMOTE_IP):$(HOST_PORT)"
	cd $(APP_DIR) && IGVI_HOST_URL=http://$(REMOTE_IP):$(HOST_PORT) $(UV) run igvi-ui
endif

app: run

# Swallow the IP argument so `make run 192.168.1.5` doesn't try to build it as a target.
%:
	@:
