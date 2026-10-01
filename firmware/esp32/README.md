\# ESP32 Firmware



Low-level firmware for the ESP32 controller used by the Autonomous Outdoor RC vehicle.



The ESP32 is responsible for real-time hardware control and sensor acquisition. It communicates with the Raspberry Pi running ROS 2 Jazzy through micro-ROS over Wi-Fi.



\## Responsibilities



\* Motor control through L298N

\* Wheel encoder acquisition

\* Steering servo control

\* BNO055 IMU acquisition

\* micro-ROS communication with the Raspberry Pi

\* Motor command timeout / safety stop



\## Hardware



\### Controller



\* ESP32

\* Arduino-ESP32 core: `2.0.17`



\### Motor



\* JGA25-371 geared DC motor

\* Built-in encoder

\* L298N motor driver



\### Sensors



\* BNO055 IMU

\* Incremental wheel encoder



\### Steering



\* Servo motor



\## Pin Configuration



| Function        | ESP32 GPIO |

| --------------- | ---------: |

| Motor IN1       |         26 |

| Motor IN2       |         27 |

| Motor PWM / ENA |         25 |

| Encoder A       |         33 |

| Encoder B       |         32 |

| Steering servo  |         14 |

| BNO055 SDA      |         21 |

| BNO055 SCL      |         22 |



\## Motor Control



The L298N is controlled using:



\* `IN1` — motor direction

\* `IN2` — motor direction

\* `ENA` — PWM speed control



PWM configuration:



\* Frequency: `20 kHz`

\* PWM channel: `4`

\* Minimum PWM: `100`

\* Command timeout: `500 ms`



If no valid motor command is received within the timeout, the motor is stopped.



\## Encoder



The wheel encoder is connected to GPIO 33 and GPIO 32.



Encoder A uses an interrupt on signal change. The accumulated encoder count is published periodically to ROS 2.



Current encoder publishing rate:



\* Approximately `20 Hz`



The encoder count is used by the ROS 2 localization system to calculate wheel odometry.



\## IMU



The BNO055 communicates over I2C:



\* SDA: GPIO 21

\* SCL: GPIO 22

\* Address: `0x28`



The firmware publishes:



\* Orientation quaternion

\* Linear acceleration

\* Angular velocity



Current IMU publishing rate:



\* Approximately `50 Hz`



Angular velocity is converted from degrees/second to radians/second for ROS compatibility.



\## Steering Servo



Servo:



\* GPIO: `14`



Current calibration:



| Position      | Value |

| ------------- | ----: |

| Maximum right |  `28` |

| Center        |  `48` |

| Maximum left  |  `68` |



These values are hardware-specific and may need recalibration if the steering mechanism changes.



\## micro-ROS



The ESP32 communicates with the Raspberry Pi using micro-ROS over Wi-Fi and UDP.



The micro-ROS agent runs on the Raspberry Pi:



```bash

ros2 run micro\_ros\_agent micro\_ros\_agent udp4 --port 8888 -v4

```



The ESP32 connects to the agent using UDP port `8888`.



The Wi-Fi network must use \*\*2.4 GHz\*\*, since the ESP32 configuration used by this project does not support 5 GHz Wi-Fi.



\## ROS 2 Topics



\### Published by ESP32



| Topic            | Message type          | Purpose             |

| ---------------- | --------------------- | ------------------- |

| `/esp32/encoder` | `std\_msgs/msg/Int32`  | Wheel encoder count |

| `/esp32/imu`     | `sensor\_msgs/msg/Imu` | BNO055 IMU data     |



\### Received by ESP32



| Topic              | Message type          | Purpose          |

| ------------------ | --------------------- | ---------------- |

| `/esp32/motor\_cmd` | project motor command | Motor control    |

| `/esp32/servo\_cmd` | project servo command | Steering control |



Exact command message definitions are maintained in the ROS 2 interface packages.



\## Building and Flashing



Open `microros.ino` in Arduino IDE.



Required libraries include:



\* Arduino-ESP32

\* micro\_ros\_arduino

\* ESP32Servo

\* Adafruit BNO055

\* Adafruit Unified Sensor



Select the appropriate ESP32 board and serial port, then upload the firmware.



After flashing, open the serial monitor and verify that the ESP32 connects to Wi-Fi and the micro-ROS agent.



\## Basic Test



Start the micro-ROS agent on the Raspberry Pi:



```bash

ros2 run micro\_ros\_agent micro\_ros\_agent udp4 --port 8888 -v4

```



Then verify the topics:



```bash

ros2 topic list

```



Check encoder data:



```bash

ros2 topic echo /esp32/encoder

```



Check IMU data:



```bash

ros2 topic echo /esp32/imu

```



\## Safety



The firmware includes a motor command timeout. If communication with the Raspberry Pi stops and no new motor command is received within the configured timeout, the motor is stopped.



Always test motor and steering commands with the vehicle lifted from the ground before performing outdoor tests.



\## Firmware Structure



```text

firmware/

└── esp32/

&#x20;   ├── README.md

&#x20;   └── microros.ino

```



\## Status



The ESP32 firmware is currently used as the low-level controller for the Autonomous Outdoor RC vehicle.



Further work includes refining motor control, encoder calibration, IMU frame orientation, steering calibration, and low-level safety behavior.



