adb root
adb wait-for-devices

adb shell rm -rf /home/linaro/optical_flow/ov-opticflow
adb shell rm -rf /home/linaro/optical_flow/opticflow

adb push ov-opticflow /home/linaro/optical_flow/
adb push opticflow /home/linaro/optical_flow

adb shell chmod 777 /home/linaro/optical_flow/opticflow
adb shell chmod 777 /home/linaro/optical_flow/ov-opticflow

adb shell sync
adb shell sync

adb reboot
