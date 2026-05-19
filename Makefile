.PHONY: help install _check-env sync host ui run app

APP_DIR := apps
UV := uv
HOST_PORT := 8770
DATA_ROOT ?= $(HOME)/igvi_robot
ENV_FILE := docker/compose/.env
ENV_EXAMPLE := docker/compose/.env.example

# `make run <ip>` connects the UI to a remote host at http://<ip>:$(HOST_PORT).
# `make run` (no IP) launches igvi-host locally and opens the UI against it.
# `make host lan` (or any non-empty arg) binds igvi-host to 0.0.0.0 so other
# machines on the LAN can connect; `make host` alone keeps the safe loopback default.
REMOTE_IP := $(filter-out run app host,$(MAKECMDGOALS))

help:
	@echo "IGVI Robot commands:"
	@echo "  make install              - One-time setup: create .env and shared data directories"
	@echo "  make install DATA_ROOT=X  - Use a custom shared data path (default: /opt/igvi_robot)"
	@echo "  make sync                 - Install/sync Python dependencies"
	@echo "  make host                 - Run igvi-host (loopback only)"
	@echo "  make host lan             - Run igvi-host bound to 0.0.0.0 (exposed on LAN)"
	@echo "  make ui                   - Run igvi-ui (local host)"
	@echo "  make run                  - Run igvi-host in the background, then open igvi-ui"
	@echo "  make run <ip>             - Open igvi-ui pointed at a remote host (http://<ip>:$(HOST_PORT))"
	@echo "  make app                  - Alias for make run"

install:
	@if [ ! -f $(ENV_FILE) ]; then \
		LC_ALL=C sed "s|IGVI_DATA_ROOT=.*|IGVI_DATA_ROOT=$(DATA_ROOT)|" $(ENV_EXAMPLE) > $(ENV_FILE) && \
		echo "  Created $(ENV_FILE) with IGVI_DATA_ROOT=$(DATA_ROOT)"; \
	else \
		echo "  $(ENV_FILE) already exists — skipping creation"; \
		echo "  Edit IGVI_DATA_ROOT in $(ENV_FILE) to change the shared data path"; \
	fi
	@DATA=$$(grep -E '^IGVI_DATA_ROOT=' $(ENV_FILE) | head -1 | cut -d= -f2-); \
	if [ -z "$$DATA" ]; then \
		echo "IGVI_DATA_ROOT=$(DATA_ROOT)" >> $(ENV_FILE) && \
		DATA=$(DATA_ROOT) && \
		echo "  Added IGVI_DATA_ROOT=$(DATA_ROOT) to $(ENV_FILE)"; \
	fi; \
	mkdir -p "$$DATA/slam" "$$DATA/maps" && \
	echo "  Data directories ready at $$DATA"

_check-env:
	@if [ ! -f $(ENV_FILE) ]; then \
		echo ""; \
		echo "  ERROR: $(ENV_FILE) not found."; \
		echo "  Run 'make install' first to set up the shared data path."; \
		echo ""; \
		exit 1; \
	fi
	@DATA=$$(grep -E '^IGVI_DATA_ROOT=' $(ENV_FILE) | head -1 | cut -d= -f2-); \
	if [ -z "$$DATA" ]; then \
		echo ""; \
		echo "  ERROR: IGVI_DATA_ROOT is not set in $(ENV_FILE)."; \
		echo "  Run 'make install' to fix this."; \
		echo ""; \
		exit 1; \
	fi; \
	if [ ! -d "$$DATA/slam" ] || [ ! -d "$$DATA/maps" ]; then \
		echo ""; \
		echo "  ERROR: Data directories missing at $$DATA"; \
		echo "  Run 'make install' to create them."; \
		echo ""; \
		exit 1; \
	fi

sync:
	cd $(APP_DIR) && $(UV) sync

host: _check-env
ifeq ($(strip $(REMOTE_IP)),)
	cd $(APP_DIR) && $(UV) run igvi-host
else
	@echo "Binding igvi-host to 0.0.0.0:$(HOST_PORT) (LAN-exposed)"
	cd $(APP_DIR) && IGVI_HOST_BIND=0.0.0.0 $(UV) run igvi-host
endif

ui:
	cd $(APP_DIR) && $(UV) run igvi-ui

run: _check-env
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
