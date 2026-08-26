#!/bin/bash
# Tests the thing that actually matters for students: killing and
# restarting ONLY the solution code (collision_detection.py +
# cleaning_robot.py, standing in for a student's own script) 8 times,
# while Webots + the webots_ros2_driver + ros2_control stack keep running
# continuously and are NEVER restarted.
set -u
CYCLES=8
SCRATCH="/private/tmp/claude-501/-Users-patric-Dropbox-Documents-claude-subsample-fit/041920fd-1e9a-46ed-a240-b12a6790a9f3/scratchpad/webots_test"

get_webots_pid() { pgrep -f "Webots.app/Contents/MacOS/webots" | head -1; }
get_driver_pid() { pgrep -f "webots_ros2_driver/lib/webots_ros2_driver/driver" | head -1; }

WEBOTS_PID_INITIAL=$(get_webots_pid)
DRIVER_PID_INITIAL=$(get_driver_pid)
echo "Initial Webots PID: $WEBOTS_PID_INITIAL  driver PID: $DRIVER_PID_INITIAL"
if [ -z "$WEBOTS_PID_INITIAL" ] || [ -z "$DRIVER_PID_INITIAL" ]; then
  echo "ABORT: Webots/driver not running -- run the sanity launch first"
  exit 1
fi

for i in $(seq 1 $CYCLES); do
  echo "===== CYCLE $i/$CYCLES: $(date +%H:%M:%S) ====="

  # Kill only the solution processes -- the student-code equivalent.
  pkill -9 -f "assignment_2_solution/lib/assignment_2_solution/collision_detection" 2>/dev/null
  pkill -9 -f "assignment_2_solution/lib/assignment_2_solution/cleaning_robot" 2>/dev/null
  sleep 1

  WEBOTS_PID_NOW=$(get_webots_pid)
  DRIVER_PID_NOW=$(get_driver_pid)
  if [ "$WEBOTS_PID_NOW" != "$WEBOTS_PID_INITIAL" ]; then
    echo "CYCLE $i: FAIL -- Webots PID changed/died ($WEBOTS_PID_INITIAL -> $WEBOTS_PID_NOW)"
  fi
  if [ "$DRIVER_PID_NOW" != "$DRIVER_PID_INITIAL" ]; then
    echo "CYCLE $i: FAIL -- driver PID changed/died ($DRIVER_PID_INITIAL -> $DRIVER_PID_NOW)"
  fi

  rm -f /tmp/solution_cycle_$i.log
  START=$(date +%s)
  bash -c "source /tmp/webots_env.sh && source '$SCRATCH/ws/install/setup.sh' && exec ros2 run assignment_2_solution collision_detection > /tmp/solution_cycle_$i.log 2>&1 &
  disown"
  bash -c "source /tmp/webots_env.sh && source '$SCRATCH/ws/install/setup.sh' && exec ros2 run assignment_2_solution cleaning_robot >> /tmp/solution_cycle_$i.log 2>&1 &
  disown"

  GOT_COLLISION=0
  for attempt in $(seq 1 20); do
    if grep -q "collision #" /tmp/solution_cycle_$i.log 2>/dev/null; then
      GOT_COLLISION=1
      break
    fi
    sleep 1
  done
  ELAPSED=$(( $(date +%s) - START ))

  if [ "$GOT_COLLISION" -eq 1 ]; then
    FIRST_COLLISION=$(grep -m1 "collision #" /tmp/solution_cycle_$i.log)
    echo "CYCLE $i: solution restarted and re-detected a collision within ${ELAPSED}s ($FIRST_COLLISION)"
  else
    echo "CYCLE $i: FAIL -- no collision re-detected within 20s of restart"
    tail -20 /tmp/solution_cycle_$i.log
  fi
done

echo "===== ALL CYCLES DONE ====="
echo "Final Webots PID: $(get_webots_pid)  (initial was $WEBOTS_PID_INITIAL)"
echo "Final driver PID: $(get_driver_pid)  (initial was $DRIVER_PID_INITIAL)"
