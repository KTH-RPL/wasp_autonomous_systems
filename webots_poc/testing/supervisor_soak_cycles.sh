#!/bin/bash
# Restart-cycle soak test for the Ros2Supervisor fix: full launch (GUI on,
# ros2_supervisor=True, use_sim_time=True) -> confirm controller connects,
# confirm /clock actually publishes real sim time (not just that the
# process didn't crash) -> full teardown -> relaunch, 5 cycles.
set -u
CYCLES=5
SCRATCH="/private/tmp/claude-501/-Users-patric-Dropbox-Documents-claude-subsample-fit/041920fd-1e9a-46ed-a240-b12a6790a9f3/scratchpad/webots_test"

for i in $(seq 1 $CYCLES); do
  echo "===== CYCLE $i/$CYCLES: $(date +%H:%M:%S) ====="
  rm -f /tmp/supervisor_soak_cycle.log
  bash -c "source /tmp/webots_env.sh && ros2 launch webots_ros2_mavic course_world_launch.py > /tmp/supervisor_soak_cycle.log 2>&1 &
  disown"

  CONNECTED=0
  for attempt in $(seq 1 30); do
    if grep -q "Controller successfully connected" /tmp/supervisor_soak_cycle.log 2>/dev/null; then
      CONNECTED=1
      break
    fi
    if grep -q "process has died.*ros2_supervisor\|Traceback" /tmp/supervisor_soak_cycle.log 2>/dev/null; then
      break
    fi
    sleep 1
  done

  if [ "$CONNECTED" -eq 0 ]; then
    echo "CYCLE $i: FAIL -- controller never connected or supervisor crashed"
    tail -30 /tmp/supervisor_soak_cycle.log
  else
    DIED=$(grep -c "process has died.*ros2_supervisor" /tmp/supervisor_soak_cycle.log 2>/dev/null || echo 0)
    rm -f /tmp/supervisor_clock_probe.log
    bash -c "source /tmp/webots_env.sh && exec ros2 topic echo /clock --once" > /tmp/supervisor_clock_probe.log 2>&1 &
    CPID=$!
    sleep 3
    kill -9 "$CPID" 2>/dev/null
    if grep -q "sec:" /tmp/supervisor_clock_probe.log; then
      CLOCK_VAL=$(grep "sec:" /tmp/supervisor_clock_probe.log | head -1)
      echo "CYCLE $i: connected, supervisor crash-count=$DIED, /clock OK ($CLOCK_VAL)"
    else
      echo "CYCLE $i: connected, supervisor crash-count=$DIED, but /clock NOT publishing"
    fi
  fi

  pkill -9 -f "course_world_launch.py" 2>/dev/null
  pkill -9 -f "webots_ros2_driver/lib/webots_ros2_driver/driver" 2>/dev/null
  pkill -9 -f "ros2_supervisor.py" 2>/dev/null
  pkill -9 -f "Webots.app/Contents/MacOS/webots" 2>/dev/null
  sleep 3

  ORPHANS=$(pgrep -f "Webots.app/Contents/MacOS/webots|webots_ros2_driver/lib/webots_ros2_driver/driver|ros2_supervisor.py" | wc -l | tr -d ' ')
  echo "orphan processes after teardown: $ORPHANS"
  echo
done

echo "===== ALL CYCLES DONE ====="
