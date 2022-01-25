# dronekit_tutorials
3DR Dronekit Tutorials for PX4 Toolchain

1. 툴체인 SITL 실행

make px4_sitl_default jmavsim
OR
make px4_sitl_default gazebo

2. 샘플 프로그램 실행

python dronekitPX4.py --connect=127.0.0.1:14540

3. 리모트 QgroundControl 접속
##If you want to connect to GCS.

mavproxy.py --master=udp:localhost:14550 --out:udp:<GCS IP>:14550
