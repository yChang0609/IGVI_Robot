#!/bin/bash

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
COMPOSE_FILE="$REPO_ROOT/docker/compose/compose.yaml"
COMPOSE_DEV_FILE="$REPO_ROOT/docker/compose/compose.dev.yaml"

cleanup() {
    echo "Shutting down docker compose services..."
    $DOCKER_COMPOSE_COMMAND -f "$COMPOSE_FILE" -f "$COMPOSE_DEV_FILE" down --timeout 0
    exit 0
}

main() {
    if docker compose version &> /dev/null; then
        DOCKER_COMPOSE_COMMAND="docker compose"
    elif command -v docker-compose &> /dev/null; then
        DOCKER_COMPOSE_COMMAND="docker-compose"
    else
        echo "Neither 'docker compose' nor 'docker-compose' is installed."
        exit 1
    fi

    if [[ ! -f "$COMPOSE_FILE" ]]; then
        echo "Error: Compose file not found: $COMPOSE_FILE" >&2
        exit 1
    fi

    local profile_args=()
    for profile in "$@"; do
        profile_args+=(--profile "$profile")
    done

    echo "Starting compose profiles: $*"
    $DOCKER_COMPOSE_COMMAND -f "$COMPOSE_FILE" -f "$COMPOSE_DEV_FILE" "${profile_args[@]}" up -d
    $DOCKER_COMPOSE_COMMAND -f "$COMPOSE_FILE" -f "$COMPOSE_DEV_FILE" logs -f &

    trap cleanup SIGINT
    wait
}
