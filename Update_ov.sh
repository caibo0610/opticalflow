adb root

adb wait-for-devices

adb shell rm /home/linaro/optical_flow/ov-opticflow
adb shell rm /home/linaro/optical_flow/opticflow

adb push ov-opticflow /home/linaro/optical_flow/
adb push opticflow /home/linaro/optical_flow/
adb push flow_camera_center_calib.txt /media/internal/calib/

adb shell chmod 777 /home/linaro/optical_flow/opticflow
adb shell chmod 777 /home/linaro/optical_flow/ov-opticflow
adb shell chmod 777 /media/internal/calib/flow_camera_center_calib.txt


adb shell sync
adb shell sync
adb shell reboot -f warm
