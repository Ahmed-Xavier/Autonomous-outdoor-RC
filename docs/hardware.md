# \# Hardware Documentation

# 

# \## Main Computer

# 

# \- Raspberry Pi 4 Model B

# \- Ubuntu 24.04

# \- ROS 2 Jazzy

# 

# The Raspberry Pi is responsible for high-level autonomy, perception, localization, planning and ROS 2 communication.

# 

# \## Low-Level Controller

# 

# \- ESP32

# 

# Responsibilities:

# 

# \- Motor control

# \- Encoder acquisition

# \- BNO055 IMU acquisition

# \- Steering servo control

# \- Low-level safety and failsafe behavior

# \- Wi-Fi micro-ROS communication

# 

# \## Motor and Driver

# 

# \### Motor

# 

# \- \*\*JGA25-370 geared DC motor\*\*

# \- Integrated incremental/quadrature encoder

# \- Motor supply: 12 V in the current wiring

# 

# \### Motor Driver

# 

# \- \*\*L298N\*\*

# 

# The current ESP32 firmware controls one motor channel:

# 

# | L298N / Motor interface | ESP32 pin |

# |---|---:|

# | IN1 | GPIO 26 |

# | IN2 | GPIO 27 |

# | ENA / PWM | GPIO 25 |

# 

# The firmware uses PWM on the ENA pin and IN1/IN2 for direction.

# 

# Current motor command range:

# 

# \- `-255` to `255`

# \- `0` = stop

# \- Positive = forward

# \- Negative = reverse

# 

# The firmware applies a minimum PWM threshold because low PWM values may not move the motor under load.

# 

# \## JGA25-370 Motor / Encoder Wiring

# 

# The current six-wire cable is:

# 

# | Wire color | Function | Connection / voltage |

# |---|---|---|

# | Yellow | Encoder A | ESP32 GPIO 33 |

# | White | Encoder B | ESP32 GPIO 32 |

# | Blue | Encoder VCC | 5 V |

# | Green | Encoder GND | GND |

# | Black | Motor negative | GND / motor supply negative |

# | Red | Motor positive | +12 V |

# 

# The encoder uses two signal channels, A and B, allowing direction to be determined from the phase relationship.

# 

# The current firmware interrupts on encoder channel A and reads channel B to determine direction.

# 

# \### Encoder ROS 2 Interface

# 

# The ESP32 publishes the accumulated encoder count on:

# 

# `/esp32/encoder`

# 

# Message type:

# 

# `std\_msgs/msg/Int32`

# 

# The current firmware publishes approximately every 50 ms, giving a nominal rate of 20 Hz.

# 

# \## Steering Servo

# 

# | Signal | ESP32 pin |

# |---|---:|

# | Servo signal | GPIO 14 |

# 

# Current firmware limits:

# 

# \- Minimum: `30`

# \- Center: `50`

# \- Maximum: `70`

# 

# These values represent the current vehicle's steering calibration and should be treated as calibration parameters rather than universal servo angles.

# 

# \## IMU

# 

# \- \*\*BNO055\*\*

# \- Planned/ongoing ESP32 integration

# 

# The BNO055 is intended to provide orientation and motion information for vehicle state estimation and sensor fusion.

# 

# \## GPS / GNSS

# 

# \- \*\*GT-U7 GPS/GNSS module\*\*

# \- Intended for outdoor position estimation and GPS waypoint navigation

# 

# Integration with the ROS 2 stack is ongoing.

# 

# \## Camera

# 

# \- \*\*Hikvision USB camera\*\*

# \- Connected to the Raspberry Pi

# \- Intended for lane detection, stop sign detection and other visual perception tasks

# 

# \## LiDAR

# 

# \- LiDAR sensor

# \- Connected to the Raspberry Pi

# \- Intended for obstacle detection, mapping and navigation

# 

# \## Power

# 

# Current motor wiring uses a 12 V motor supply.

# 

# The encoder is powered separately at 5 V through its encoder VCC/GND pair.

# 

# Power distribution and common-ground connections should be verified against the exact battery, regulator and vehicle wiring before final assembly.

# 

# \## Hardware Summary

# 

# | Component | Model | Main role |

# |---|---|---|

# | Computer | Raspberry Pi 4 Model B | High-level autonomy |

# | Controller | ESP32 | Low-level control |

# | Motor | JGA25-370 | Propulsion |

# | Motor driver | L298N | Motor power/direction control |

# | Encoder | JGA25-370 integrated encoder | Motion feedback |

# | GPS | GT-U7 | Outdoor localization |

# | IMU | BNO055 | Orientation/motion |

# | Camera | Hikvision USB | Visual perception |

# | LiDAR | Current project LiDAR | Obstacle detection |

# | Steering | RC servo | Steering |

# 

# \## Important Pinout Reference

# 

# \### ESP32

# 

# | Function | GPIO |

# |---|---:|

# | L298N IN1 | 26 |

# | L298N IN2 | 27 |

# | L298N ENA/PWM | 25 |

# | Encoder A | 33 |

# | Encoder B | 32 |

# | Steering servo | 14 |

# 

# These values match `firmware/esp32/microros.ino`.

