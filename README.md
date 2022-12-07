# ESPServoControllerRos2
### PwmConverterRC2ESP.ino (Sample program without ros2)
RCレシーバーのPWM波(2チャンネル)をESP32を用いて測定し，同じ立ち上がり長さのPWM波を出力するプログラムです．

This program measures the PWM wave (2 channel) of the RC receiver using ESP32 and outputs a PWM wave with the same rise length.

### TwistToPWMServo.ino (Sample program with ros2)
micro-rosを用いてESP32でTwistトピックをサブスクライブし，サーボにPWM波を出力するプログラムです．

This program subscribes Twist topic on ESP32 using micro-ros and outputs PWM wave to the servo.

Connect micro-ros-agent to ros2-for-arduino

If USB (/dev/ttyUSB0)

$ ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6

If Wifi

$ ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6

Controll ros2 with micro-ros-agent PC's keyborad

$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/micro_ros_arduino_twist_subscriber

