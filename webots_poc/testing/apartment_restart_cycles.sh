#!/bin/bash
# Full restart-cycle soak test for Assignment 1 (RGB-D camera) on native
# Webots: 8 cycles of full launch (GUI on, apartment world) -> verify
# camera/point-cloud/encoders are actually streaming -> full teardown ->
# relaunch. Mirrors the Assignment 2 methodology.
set -u
CYCLES=8

for i in $(seq 1 $CYCLES); do
  echo "===== CYCLE $i/$CYCLES: $(date +%H:%M:%S) ====="

  ORPHANS_BEFORE=$(pgrep -f "Webots.app/Contents/MacOS/webots|webots_ros2_driver/lib/webots_ros2_driver/driver|controller_manager/spawner" | wc -l | tr -d ' ')
  echo "orphan processes before launch: $ORPHANS_BEFORE"

  rm -f /tmp/apartment_launch_cycle.log
  LAUNCH_START=$(date +%s)
  bash -c "source /tmp/webots_env.sh && ros2 launch assignment_1_solution apartment_rgbd_launch.py > /tmp/apartment_launch_cycle.log 2>&1 &
  disown"

  CONNECTED=0
  for attempt in $(seq 1 40); do
    if grep -q "Controller successfully connected" /tmp/apartment_launch_cycle.log 2>/dev/null; then
      CONNECTED=1
      break
    fi
    sleep 1
  done

  ACTIVATED=0
  if [ "$CONNECTED" -eq 1 ]; then
    for attempt in $(seq 1 20); do
      if [ "$(grep -c 'Configured and activated' /tmp/apartment_launch_cycle.log 2>/dev/null)" -ge 2 ]; then
        ACTIVATED=1
        break
      fi
      if grep -q "Switch controller timed out\|Failed to activate controller" /tmp/apartment_launch_cycle.log 2>/dev/null; then
        break
      fi
      sleep 1
    done
  fi
  LAUNCH_ELAPSED=$(( $(date +%s) - LAUNCH_START ))

  if [ "$CONNECTED" -eq 0 ]; then
    echo "CYCLE $i: FAILED TO CONNECT within 40s (took ${LAUNCH_ELAPSED}s)"
    tail -20 /tmp/apartment_launch_cycle.log
  elif [ "$ACTIVATED" -eq 0 ]; then
    echo "CYCLE $i: connected in ${LAUNCH_ELAPSED}s but CONTROLLERS FAILED TO ACTIVATE"
    tail -20 /tmp/apartment_launch_cycle.log
  else
    echo "CYCLE $i: connected + controllers activated in ${LAUNCH_ELAPSED}s"
    bash /tmp/apartment_sensor_check.sh
  fi

  pkill -9 -f "apartment_rgbd_launch.py" 2>/dev/null
  pkill -9 -f "webots_ros2_driver/lib/webots_ros2_driver/driver" 2>/dev/null
  pkill -9 -f "controller_manager/spawner" 2>/dev/null
  pkill -9 -f "assignment_1_solution/lib/assignment_1_solution/encoders" 2>/dev/null
  pkill -9 -f "Webots.app/Contents/MacOS/webots" 2>/dev/null
  sleep 3

  ORPHANS_AFTER=$(pgrep -f "Webots.app/Contents/MacOS/webots|webots_ros2_driver/lib/webots_ros2_driver/driver|controller_manager/spawner" | wc -l | tr -d ' ')
  echo "orphan processes after teardown: $ORPHANS_AFTER"
  if [ "$ORPHANS_AFTER" -gt 0 ]; then
    echo "CYCLE $i: WARNING - processes survived teardown:"
    pgrep -fl "Webots.app/Contents/MacOS/webots|webots_ros2_driver/lib/webots_ros2_driver/driver|controller_manager/spawner"
  fi

  echo
done

echo "===== ALL CYCLES DONE ====="
