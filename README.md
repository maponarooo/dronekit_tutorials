# dronekit_tutorials
3DR Dronekit Tutorials for PX4 Toolchain

make px4_sitl_default jmavsim
OR
make px4_sitl_default gazebo

##If you want to connect to GCS.
mavproxy.py --master=udp:localhost:14550 --out:udp:<GCS IP>:14550
