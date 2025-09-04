#!/bin/bash

header="┏━━━━━━━━━━━━━━━━━  \e[5m\e[4mPLEASE NOTE\e[24m\e[25m ━━━━━━━━━━━━━━━━━┓"
filler="┃                                                ┃"
footer="┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛"
text_1="┃         Cannot find shared directory.          ┃"
text_2="┃         Please mount shared directory.         ┃"
text_3="┃         \033[1;34m\e]8;;https://github.com/KTH-RPL/wasp_autonomous_systems/wiki/Setup-Multipass-VM#create-and-mount-shared-directory\aLink to Wiki for instructions\e]8;;\a\033[1;31m          ┃"

if [ -d /home/ubuntu/shared ]; then
  host_dir=$(mount | grep '/home/ubuntu/shared' | sed -rn 's/:(.*)\son\s\/home\/ubuntu\/shared.*/\1/p')
  if [[ "$host_dir" =~ ^[a-zA-Z]:.* ]]; then
    # Windows host
    host_dir=$(echo "$host_dir" | sed -re 's/\\/\//g' -e 's/(.*):/\/mnt\/\1/')
  fi
  host_dir=$(echo $host_dir | sed -re 's/\s/\\ /g')

  if [ -z "${host_dir}" ]; then
    echo -e "\n\033[1;31m$header\n$filler\n$text_1\n$text_2\n$filler\n$text_3\n$filler\n$footer\033[0m\n"
  else
    echo "Extracting rosbags"
    cd /home/ubuntu/shared
    pv /home/ubuntu/rosbags.tar.zst | tar -I "unzstd -T0" -x
    cd rosbags
    echo "Verifying checksums"
    if md5sum -c /home/ubuntu/rosbags_md5sums.txt; then
      echo -e "\033[1;32m✅ Successfully extracted the rosbags! ✅\033[0m"
    else
      echo -e "\033[1;31m❌ Failed to extract the rosbags! ❌\033[0m"
    fi
  fi
else
  echo -e "\n\033[1;31m$header\n$filler\n$text_1\n$text_2\n$filler\n$text_3\n$filler\n$footer\033[0m\n"
fi