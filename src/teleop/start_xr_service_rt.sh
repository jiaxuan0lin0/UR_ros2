#!/usr/bin/env bash
set -euo pipefail

SERVICE_DIR="${XR_SERVICE_DIR:-/opt/apps/roboticsservice}"
RT_PRIORITY="${XR_SERVICE_RT_PRIORITY:-60}"
LOG_FILE="${XR_SERVICE_LOG_FILE:-/tmp/xrobotics_service.log}"
RESTART=0

for arg in "$@"; do
  case "$arg" in
    --restart)
      RESTART=1
      ;;
    --help|-h)
      cat <<'EOF'
Usage: src/teleop/start_xr_service_rt.sh [--restart]

Starts or boosts XRoboToolkit PC Service with SCHED_FIFO priority.

Environment variables:
  XR_SERVICE_DIR          Service directory, default /opt/apps/roboticsservice
  XR_SERVICE_RT_PRIORITY  RT priority, default 60
  XR_SERVICE_LOG_FILE     Log file, default /tmp/xrobotics_service.log
EOF
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg" >&2
      exit 2
      ;;
  esac
done

if [[ ! -x "$SERVICE_DIR/RoboticsServiceProcess" ]]; then
  echo "RoboticsServiceProcess not found or not executable in $SERVICE_DIR" >&2
  exit 1
fi

boost_pid() {
  local pid="$1"
  if command -v chrt >/dev/null 2>&1; then
    chrt -f -p "$RT_PRIORITY" "$pid" || {
      echo "Warning: failed to set SCHED_FIFO priority $RT_PRIORITY for PID $pid" >&2
    }
  fi
}

find_service_pids() {
  pgrep -x RoboticsService 2>/dev/null || true
}

if [[ "$RESTART" == "1" ]]; then
  pkill -x RoboticsService 2>/dev/null || true
  sleep 0.5
fi

mapfile -t pids < <(find_service_pids)

if [[ "${#pids[@]}" -eq 0 ]]; then
  export LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}:$SERVICE_DIR:$SERVICE_DIR/lib:$SERVICE_DIR/SDK/x64"
  export QT_PLUGIN_PATH="$SERVICE_DIR/plugins/:${QT_PLUGIN_PATH:-}"
  export QT_QML_PATH="$SERVICE_DIR/qml/:${QT_QML_PATH:-}"
  cd "$SERVICE_DIR"
  if command -v chrt >/dev/null 2>&1; then
    chrt -f "$RT_PRIORITY" ./RoboticsServiceProcess >"$LOG_FILE" 2>&1 &
  else
    ./RoboticsServiceProcess >"$LOG_FILE" 2>&1 &
  fi
  sleep 0.5
  mapfile -t pids < <(find_service_pids)
else
  for pid in "${pids[@]}"; do
    boost_pid "$pid"
  done
fi

if [[ "${#pids[@]}" -eq 0 ]]; then
  echo "Failed to start RoboticsServiceProcess. See $LOG_FILE" >&2
  exit 1
fi

ps -o pid,pri,rtprio,cls,ni,cmd -p "$(IFS=,; echo "${pids[*]}")"
