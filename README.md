# ESPServoControllerRos2
### PwmConverterRC2ESP.ino (Sample program without ros2)
RCレシーバーのPWM波(2チャンネル)をESP32を用いて測定し，同じ立ち上がり長さのPWM波を出力するプログラムです．

This program measures the PWM wave (2 channel) of the RC receiver using ESP32 and outputs a PWM wave with the same rise length.

### TwistToPWMServo.ino
micro-rosを用いてESP32でTwistトピックをサブスクライブし，サーボにPWM波を出力するプログラムです．

This program subscribes Twist topic with ESP32 using micro-ros and outputs PWM wave to the servo.
