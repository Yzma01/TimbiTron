#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$ROOT_DIR/.venv"
PYTHON_BIN="${PYTHON_BIN:-python3}"
STATE_DIR="$ROOT_DIR/.bgraph"
PID_FILE="$STATE_DIR/backend.pid"
LOG_FILE="$STATE_DIR/backend.log"

mkdir -p "$STATE_DIR"

ensure_venv() {
    if [[ ! -x "$VENV_DIR/bin/python" ]]; then
        echo "Missing virtualenv. Run: ./bgraph.sh setup"
        exit 1
    fi
}

run_python() {
    ensure_venv
    PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}" "$VENV_DIR/bin/python" "$@"
}

backend_pid() {
    if [[ -f "$PID_FILE" ]]; then
        cat "$PID_FILE"
    fi
}

backend_running() {
    local pid
    pid="$(backend_pid || true)"
    [[ -n "${pid:-}" ]] && kill -0 "$pid" 2>/dev/null
}

require_backend_stopped() {
    if backend_running; then
        echo "Stop the background backend first. Current PID: $(backend_pid)"
        exit 1
    fi
}

cmd_setup() {
    "$PYTHON_BIN" -m venv "$VENV_DIR"
    "$VENV_DIR/bin/pip" install -r "$ROOT_DIR/requirements.txt"
}

cmd_backend() {
    require_backend_stopped
    run_python -i "$ROOT_DIR/bg.py" "$@"
}

cmd_backend_virtual() {
    require_backend_stopped
    BGRAPH_VIRTUAL=1 run_python -i "$ROOT_DIR/bg.py" --virtual
}

cmd_backend_wifi() {
    require_backend_stopped
    local host="${1:-${BGRAPH_ESP32_HOST:-}}"
    local port="${2:-${BGRAPH_ESP32_PORT:-8888}}"
    if [[ -z "$host" ]]; then
        echo "Usage: ./bgraph.sh backend-wifi <host> [port]"
        exit 1
    fi
    BGRAPH_ESP32_HOST="$host" BGRAPH_ESP32_PORT="$port" run_python -i "$ROOT_DIR/bg.py" --host "$host" --tcp-port "$port"
}

cmd_esp_test() {
    if backend_running; then
        echo "Stop the backend before running esp-test. Current PID: $(backend_pid)"
        exit 1
    fi
    if [[ $# -gt 1 && "$1" == "--host" ]]; then
        BGRAPH_ESP32_HOST="$2" run_python "$ROOT_DIR/test_esp32.py" --host "$2"
    elif [[ $# -gt 0 ]]; then
        BGRAPH_SERIAL_PORT="$1" run_python "$ROOT_DIR/test_esp32.py" --port "$1"
    else
        run_python "$ROOT_DIR/test_esp32.py"
    fi
}

cmd_esp_test_wifi() {
    local host="${1:-${BGRAPH_ESP32_HOST:-}}"
    local port="${2:-${BGRAPH_ESP32_PORT:-8888}}"
    if backend_running; then
        echo "Stop the backend before running esp-test. Current PID: $(backend_pid)"
        exit 1
    fi
    if [[ -z "$host" ]]; then
        echo "Usage: ./bgraph.sh esp-test-wifi <host> [port]"
        exit 1
    fi
    BGRAPH_ESP32_HOST="$host" BGRAPH_ESP32_PORT="$port" run_python "$ROOT_DIR/test_esp32.py" --host "$host" --tcp-port "$port"
}

cmd_start() {
    ensure_venv
    if backend_running; then
        echo "Backend already running with PID $(backend_pid)"
        return 0
    fi
    (
        cd "$ROOT_DIR"
        setsid env PYTHONPATH="$ROOT_DIR${PYTHONPATH:+:$PYTHONPATH}" \
            "$VENV_DIR/bin/python" -c \
            "import os, threading, bg; plotter = bg.build_bg(os.environ.get('BGRAPH_SERIAL_PORT'), os.environ.get('BGRAPH_VIRTUAL') == '1', os.environ.get('BGRAPH_ESP32_HOST'), int(os.environ.get('BGRAPH_ESP32_PORT', '8888'))); print('BrachioGraph backend ready', flush=True); threading.Event().wait()" \
            >>"$LOG_FILE" 2>&1 </dev/null &
        echo $! >"$PID_FILE"
    )
    sleep 1
    if backend_running; then
        echo "Backend started with PID $(backend_pid)"
    else
        echo "Backend failed to stay up. Check: ./bgraph.sh logs"
        return 1
    fi
}

cmd_start_wifi() {
    local host="${1:-${BGRAPH_ESP32_HOST:-}}"
    local port="${2:-${BGRAPH_ESP32_PORT:-8888}}"
    if [[ -z "$host" ]]; then
        echo "Usage: ./bgraph.sh start-wifi <host> [port]"
        exit 1
    fi
    BGRAPH_ESP32_HOST="$host" BGRAPH_ESP32_PORT="$port" cmd_start
}

cmd_stop() {
    if ! backend_running; then
        rm -f "$PID_FILE"
        echo "Backend is not running"
        return 0
    fi
    kill "$(backend_pid)"
    rm -f "$PID_FILE"
    echo "Backend stopped"
}

cmd_restart() {
    cmd_stop
    cmd_start
}

cmd_status() {
    if backend_running; then
        echo "Backend running with PID $(backend_pid)"
    else
        rm -f "$PID_FILE"
        echo "Backend stopped"
        return 1
    fi
}

cmd_logs() {
    if [[ -f "$LOG_FILE" ]]; then
        tail -n 40 "$LOG_FILE"
    else
        echo "No backend log yet"
    fi
}

cmd_port() {
    run_python -c "import bg; print(bg.detect_serial_port() or 'NO_PORT_FOUND')"
}

usage() {
    cat <<'EOF'
Usage: ./bgraph.sh <command>

Commands:
  setup             Create .venv and install Python dependencies
  backend [args]    Open an interactive backend shell
  backend-wifi H P  Open an interactive backend shell over WiFi
  backend-virtual   Open an interactive backend shell in virtual mode
  esp-test [port]   Run the ESP32 serial smoke test
  esp-test-wifi H P Run the ESP32 WiFi smoke test
  start             Start the backend in the background
  start-wifi H P    Start the backend in the background over WiFi
  stop              Stop the background backend
  restart           Restart the background backend
  status            Show backend status
  logs              Show backend log output
  port              Detect the Linux serial port the backend would use
EOF
}

main() {
    local command="${1:-}"
    shift || true

    case "$command" in
        setup) cmd_setup ;;
        backend) cmd_backend "$@" ;;
        backend-wifi) cmd_backend_wifi "$@" ;;
        backend-virtual) cmd_backend_virtual ;;
        esp-test) cmd_esp_test "$@" ;;
        esp-test-wifi) cmd_esp_test_wifi "$@" ;;
        start) cmd_start ;;
        start-wifi) cmd_start_wifi "$@" ;;
        stop) cmd_stop ;;
        restart) cmd_restart ;;
        status) cmd_status ;;
        logs) cmd_logs ;;
        port) cmd_port ;;
        *) usage; exit 1 ;;
    esac
}

main "$@"
