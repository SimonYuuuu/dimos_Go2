#!/usr/bin/env bash
# Stop a background DimOS (patrol/mapping) and the robot agent on this Orin.
# Mapping-only (keep agent): ./simulation_legacy/stop_mapping.sh
# WebRTC drops when DimOS exits, so the dog lies down.
set -u

echo "Stopping DimOS and the robot agent. The dog will lie down."

stop_match() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -TERM -f "$pattern" >/dev/null 2>&1 || true
  fi
}

# The shell first, so it cannot start DimOS again after the child exits.
stop_match '[s]tart_mapping_quality\.sh'
stop_match '[s]tart_mapping\.sh'
stop_match '[s]tart_patrol.sh'
stop_match '[d]imos --viewer'
stop_match '[d]imos run'
stop_match '[s]tart_robot_agent.sh'
stop_match '[/]robot_agent/agent.py'
stop_match 'dimos.core.coordination.watchdog_main'
sleep 2

force_match() {
  local pattern="$1"
  if pgrep -f "$pattern" >/dev/null 2>&1; then
    pkill -KILL -f "$pattern" >/dev/null 2>&1 || true
  fi
}

force_match '[s]tart_mapping_quality\.sh'
force_match '[s]tart_mapping\.sh'
force_match '[s]tart_patrol.sh'
force_match '[d]imos --viewer'
force_match '[d]imos run'
force_match '[s]tart_robot_agent.sh'
force_match '[/]robot_agent/agent.py'
force_match 'dimos.core.coordination.watchdog_main'
force_match 'simulation_legacy/.venv/bin/python -c from multiprocessing.forkserver'
force_match 'simulation_legacy/.venv/bin/python -c from multiprocessing.resource_tracker'
sleep 0.5

left="$(pgrep -af '[d]imos --viewer|[s]tart_patrol.sh|[s]tart_mapping|[\/]robot_agent/agent.py|[s]tart_robot_agent.sh' || true)"
if [[ -n "$left" ]]; then
  echo "Some processes are still running:"
  echo "$left"
  exit 1
fi

echo "DimOS and the robot agent are stopped."
