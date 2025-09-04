#!/bin/bash

function cleanup {
  sed -i "s/WAIT ⚠️/FAIL ❌ \[Never executed\]/g" $log_file
}

trap cleanup EXIT

log_file=$1

log_info()
{
  local start=$EPOCHREALTIME

  eval "$@"
  ret=$?

  local end=$EPOCHREALTIME
  local duration=$(echo "scale=2;($end-$start)/1" | bc)

  if [ $ret -eq 0 ]; then
    sed -i "0,/WAIT ⚠️/s//PASS ✅ \[$duration s\]/" $log_file
  else
    sed -i "0,/WAIT ⚠️/s//FAIL ❌ \[$duration s\] \[Error code: $ret\]/" $log_file

    # Terminate the script early
    exit $ret
  fi
}

# Download rosbags in background    
curl -L -C - https://canvas.kth.se/files/9471655/download\?download_frd\=1\&verifier\=4wOMCwu3QHS0duQGTKpn6HyhO9jCPGZSL3RM9kmu --output /home/ubuntu/rosbags.tar.zst &
ROSBAGS_DOWNLOAD_PID=$!
    
log_info "mkdir /home/ubuntu/.config"
    
log_info "git clone --recurse-submodules https://github.com/KTH-RPL/wasp_autonomous_systems.git -b ht25 /home/ubuntu/ros2_ws/src/wasp_autonomous_systems"

log_info "python3 -m venv /home/ubuntu/ros2_ws/venv --system-site-packages && touch /home/ubuntu/ros2_ws/venv/COLCON_IGNORE"
    
log_info ". /home/ubuntu/ros2_ws/venv/bin/activate"
    
log_info "python3 -m pip install --upgrade pip && python3 -m pip install -r /home/ubuntu/ros2_ws/src/wasp_autonomous_systems/requirements.txt"
    
mkdir /home/ubuntu/yolo_models

log_info "cd /home/ubuntu/yolo_models && yolo segment export model=yolo11n-seg.pt format=openvino half=True device=cpu dynamic=True && yolo segment export model=yolo11n-seg.pt format=openvino int8=True device=cpu dynamic=True data=coco128-seg.yaml"
    
log_info ". /opt/ros/jazzy/setup.bash && cd /home/ubuntu/ros2_ws && python3 -m colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install --packages-up-to-regex assignment"

for i in 1 2 3 4; do
  log_info "test -d '/home/ubuntu/ros2_ws/install/assignment_$i'"
done

log_info "wait $ROSBAGS_DOWNLOAD_PID"

log_info "echo db0c7fe93e4b78649c95a5f5b4e7dec4  /home/ubuntu/rosbags.tar.zst | md5sum -c -"