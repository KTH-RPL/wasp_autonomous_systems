# Rosbags

## Download Original Rosbags

Download the original `kitti` and `real_robot` rosbags and put them in the folder `original_rosbags_dir`.

```sh
original_rosbags_dir="..."
```

## Create Small Rosbags

Compress the image topics using jpeg. Store the new `kitti` and `real_robot` rosbags in the folder `new_rosbags_dir`.

```sh
new_rosbags_dir="rosbags"
```

### kitti

```sh
in_kitti_dir="$original_rosbags_dir/kitti"
out_kitti_dir="$new_rosbags_dir/kitti"

ros2 bag play --read-ahead-queue-size 100 --topics /kitti/camera/color/left/camera_info /kitti/camera/color/left/image /kitti/camera/color/right/camera_info /kitti/camera/color/right/image /kitti/oxts/gps/fix /kitti/oxts/gps/vel /kitti/oxts/imu /kitti/velo /tf /tf_static --clock-topics-all -p "$in_kitti_dir"

ros2 bag record --topics /kitti/camera/color/left/camera_info /kitti/camera/color/left/image/compressed /kitti/camera/color/right/camera_info /kitti/camera/color/right/image/compressed /kitti/oxts/gps/fix /kitti/oxts/gps/vel /kitti/oxts/imu /kitti/velo /tf /tf_static --use-sim-time -o "$out_kitti_dir"

ros2 run image_transport republish --ros-args -p use_sim_time:=true -p in_transport:="raw" -p out_transport:="compressed" --remap in:=/kitti/camera/color/left/image --remap out/compressed:=/kitti/camera/color/left/image/compressed

ros2 run image_transport republish --ros-args -p use_sim_time:=true -p in_transport:="raw" -p out_transport:="compressed" --remap in:=/kitti/camera/color/right/image --remap out/compressed:=/kitti/camera/color/right/image/compressed
```

### real_robot

```sh
in_real_robot_dir="$original_rosbags_dir/real_robot"
out_real_robot_dir="$new_rosbags_dir/real_robot"

ros2 bag play --read-ahead-queue-size 100 --topics /camera/color/camera_info /camera/color/image_raw /camera/depth/color/points /motor/duty_cyles /motor/encoders /tf_static --clock-topics-all -p "$in_real_robot_dir"

ros2 bag record --topics /camera/color/camera_info /camera/color/image/compressed /camera/depth/color/points /motor/duty_cyles /motor/encoders /tf_static --use-sim-time -o "$out_real_robot_dir"

ros2 run image_transport republish --ros-args -p use_sim_time:=true -p in_transport:="raw" -p out_transport:="compressed" --remap in:=/camera/color/image_raw --remap out/compressed:=/camera/color/image/compressed
```

## Split the Rosbags

Split the bags into 500 MiB chunks (if a file is larger than 2 GiB it can cause problems on some filesystems. It does not work with a Multipass mount using type `classic` on Windows for example).

### kitti

Create a file `out.yaml` with content:

```sh
output_bags:
- uri: kitti
  max_bagfile_size: 524288000
  all_topics: true
  all_services: true
```

run:

```sh
ros2 bag convert -i kitti2 -o out.yaml
```

where `kitti2` is the non-split rosbag.

### real_robot

Create a file `out.yaml` with content:

```sh
output_bags:
- uri: real_robot
  max_bagfile_size: 524288000
  all_topics: true
  all_services: true
```

run:

```sh
ros2 bag convert -i real_robot2 -o out.yaml
```

where `real_robot2` is the non-split rosbag.

## Compress the Rosbags

Using Zstd with maximum compression. This will be slow to compress, but the file will be small and still fast to decompress.

```sh
in_rosbags_dir="$new_rosbags_dir"
out_compressed_file="rosbags.tar.zst"

tar -I "zstd -T0 --auto-threads=logical --long --ultra -22 --progress" -cf "$out_compressed_file" "$in_rosbags_dir"
```

## Checksums

Generate checksums

```sh
cd "$new_rosbags_dir"
find . -type f -exec md5sum {} > md5sums.txt \;
```

Open the file and look for the md5sum for `./rosbags.tar.zst` and update the md5sum correspondinly in `wasp.yaml`.

Also update the `./real_robot/*` and `./kitti/*` md5sums in the same way.

## Decompress the Rosbags

Install pv to get Progress

```sh
sudo apt install pv
```

Decompress

```sh
in_file="rosbags.tar.zst"
out_folder="rosbags"

pv "$in_file" | tar -I "unzstd -T0" -x -C "$out_folder"
```
