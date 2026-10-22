#!/usr/bin/env bash

WS_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/../.." && pwd)"
WITH_VENV="$WS_DIR/scripts/with_venv.sh"

TIMEOUT_SEC="${TIMEOUT_SEC:-4}"
FAILURES=0

print_header() {
  printf '\n== %s ==\n' "$1"
}

ok() {
  printf '  [OK] %s\n' "$1"
}

warn() {
  printf '  [WARN] %s\n' "$1"
}

fail() {
  printf '  [FAIL] %s\n' "$1"
  FAILURES=$((FAILURES + 1))
}

ros_topic_list() {
  timeout "$TIMEOUT_SEC" "$WITH_VENV" ros2 topic list 2>/dev/null
}

topic_exists() {
  local topic="$1"
  ros_topic_list | grep -Fxq "$topic"
}

require_topic() {
  local topic="$1"
  if topic_exists "$topic"; then
    ok "$topic exists"
  else
    fail "$topic missing"
  fi
}

optional_topic() {
  local topic="$1"
  if topic_exists "$topic"; then
    ok "$topic exists"
  else
    warn "$topic missing"
  fi
}

check_topic_hz() {
  local topic="$1"
  local label="${2:-$topic}"

  if ! topic_exists "$topic"; then
    fail "$label hz unavailable: topic missing"
    return
  fi

  local output
  output="$(timeout "$TIMEOUT_SEC" "$WITH_VENV" ros2 topic hz "$topic" 2>/dev/null || true)"
  if echo "$output" | grep -q "average rate"; then
    ok "$label $(echo "$output" | grep "average rate" | tail -1)"
  else
    warn "$label hz not measured within ${TIMEOUT_SEC}s"
  fi
}

check_topic_once() {
  local topic="$1"
  local label="${2:-$topic}"
  local qos_args=()

  if [[ "${3:-}" == "transient_local" ]]; then
    qos_args=(--qos-durability transient_local)
  fi

  if ! topic_exists "$topic"; then
    fail "$label echo unavailable: topic missing"
    return
  fi

  if timeout "$TIMEOUT_SEC" "$WITH_VENV" ros2 topic echo "$topic" --once "${qos_args[@]}" >/dev/null 2>&1; then
    ok "$label produced one message"
  else
    warn "$label did not produce one message within ${TIMEOUT_SEC}s"
  fi
}

check_frame_id() {
  local topic="$1"
  local expected="$2"
  local durability="${3:-volatile}"
  local qos_args=()

  if [[ "$durability" == "transient_local" ]]; then
    qos_args=(--qos-durability transient_local)
  fi

  if ! topic_exists "$topic"; then
    fail "$topic frame check unavailable: topic missing"
    return
  fi

  local frame
  frame="$(timeout "$TIMEOUT_SEC" "$WITH_VENV" ros2 topic echo "$topic" --once --field header.frame_id "${qos_args[@]}" 2>/dev/null | tr -d "'\"" | head -1 || true)"
  if [[ -z "$frame" ]]; then
    warn "$topic frame_id not read within ${TIMEOUT_SEC}s"
  elif [[ "$frame" == "$expected" ]]; then
    ok "$topic frame_id=$frame"
  else
    fail "$topic frame_id=$frame expected=$expected"
  fi
}

check_tf() {
  local parent="$1"
  local child="$2"

  if timeout "$TIMEOUT_SEC" "$WITH_VENV" ros2 run tf2_ros tf2_echo "$parent" "$child" >/dev/null 2>&1; then
    ok "TF $parent -> $child available"
  else
    fail "TF $parent -> $child unavailable"
  fi
}

finish_report() {
  if [[ "$FAILURES" -eq 0 ]]; then
    printf '\nResult: PASS\n'
    exit 0
  fi

  printf '\nResult: FAIL (%d issue(s))\n' "$FAILURES"
  exit 1
}
