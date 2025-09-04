
[[ -z "${COLCON_HOME}" ]] && export COLCON_HOME="$HOME/.colcon"

[[ -z "${XAUTHORITY}" ]] && export XAUTHORITY=$HOME/.Xauthority
    
# Source Python virtual environment
if [ -f /home/ubuntu/ros2_ws/venv/bin/activate ]; then
  source /home/ubuntu/ros2_ws/venv/bin/activate
fi

# Source ROS
if [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

if [ -f /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash ]; then
  source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
fi

# Source local ROS workspace
if [ -f /home/ubuntu/ros2_ws/install/local_setup.bash ]; then
  source /home/ubuntu/ros2_ws/install/local_setup.bash
fi

# Alias for building ROS workspace with symbolic links
alias cbs="colcon build --symlink-install"

# Make sure that you can execute the newly installed programs by adding your local bin folder to the PATH
export PATH="/home/ubuntu/.local/bin:$PATH"

# Make sure ROS only communicates over localhost
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
    
# Tell Webots what shared folder to use

header="┏━━━━━━━━━━━━━━━━━  \e[5m\e[4mPLEASE NOTE\e[24m\e[25m ━━━━━━━━━━━━━━━━━┓"
filler="┃                                                ┃"
footer="┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛"
text_1="┃         Cannot find shared directory.          ┃"
text_2="┃         Please mount shared directory.         ┃"
text_3="┃         \033[1;34m\e]8;;https://github.com/KTH-RPL/wasp_autonomous_systems/wiki/Setup-Multipass-VM#create-and-mount-shared-directory\aLink to Wiki for instructions\e]8;;\a\033[1;31m          ┃"
    
if [ -d /home/ubuntu/shared ]; then
  if [ ! -d /home/ubuntu/shared/webots ]; then
    mkdir /home/ubuntu/shared/webots
  fi
  host_dir=$(mount | grep '/home/ubuntu/shared' | sed -rn 's/:(.*)\son\s\/home\/ubuntu\/shared.*/\1/p')
  if [[ "$host_dir" =~ ^[a-zA-Z]:.* ]]; then
    # Windows host
    host_dir=$(echo "$host_dir" | sed -re 's/\\/\//g' -e 's/(.*):/\/mnt\/\1/')
  fi
  host_dir=$(echo $host_dir | sed -re 's/\s/\\ /g')
  if [ -z "${host_dir}" ]; then
    echo -e "\n\033[1;31m$header\n$filler\n$text_1\n$text_2\n$filler\n$text_3\n$filler\n$footer\033[0m\n"
  fi
  export WEBOTS_SHARED_FOLDER="$host_dir/webots":/home/ubuntu/shared/webots
else
  echo -e "\n\033[1;31m$header\n$filler\n$text_1\n$text_2\n$filler\n$text_3\n$filler\n$footer\033[0m\n"
fi

# Tell where YOLO models are located
export YOLO_MODELS_DIR="/home/ubuntu/yolo_models"