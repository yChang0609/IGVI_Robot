ros2 run image_transport republish --ros-args -p in_transport:=compressed -p out_transport:=raw --remap in/compressed:=/camera/depth/compressed --remap out:=/camera/depth/image_raw &
sleep 2
ros2 topic info /camera/depth/image_raw
kill %1
