#!/bin/bash
# Checks that the RGB-D camera/point-cloud/encoders topics are actually
# streaming data (not just that the topics exist), by sampling ros2 topic hz
# for a few seconds. Prints a single parseable line.
set -u
bash -c "
source /tmp/webots_env.sh
ros2 topic hz /camera/color/image_raw > /tmp/hz_color_cycle.log 2>&1 &
HZ1=\$!
ros2 topic hz /camera/color/points > /tmp/hz_points_cycle.log 2>&1 &
HZ2=\$!
sleep 5
kill \$HZ1 \$HZ2 2>/dev/null
ros2 topic echo /motor/encoders --once > /tmp/enc_cycle.log 2>&1 &
EPID=\$!
sleep 2
kill \$EPID 2>/dev/null
"

COLOR_RATE=$(grep -o "average rate: [0-9.]*" /tmp/hz_color_cycle.log 2>/dev/null | tail -1 | awk '{print $3}')
POINTS_RATE=$(grep -o "average rate: [0-9.]*" /tmp/hz_points_cycle.log 2>/dev/null | tail -1 | awk '{print $3}')
ENC_OK=$(grep -q "encoder_left" /tmp/enc_cycle.log 2>/dev/null && echo 1 || echo 0)

if [ -n "${COLOR_RATE:-}" ] && [ -n "${POINTS_RATE:-}" ] && [ "$ENC_OK" = "1" ]; then
  echo "SENSOR_CHECK_RESULT=PASS color_hz=$COLOR_RATE points_hz=$POINTS_RATE encoders_ok=$ENC_OK"
else
  echo "SENSOR_CHECK_RESULT=FAIL color_hz=${COLOR_RATE:-none} points_hz=${POINTS_RATE:-none} encoders_ok=$ENC_OK"
fi
