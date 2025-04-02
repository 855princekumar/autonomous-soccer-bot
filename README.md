# Autonomous Robo Soccer Robot

This repository contains the design and implementation of an autonomous robotic soccer player capable of operating in both manual and autonomous modes. The robot is designed to participate in soccer matches, with functionalities to control movement and execute kicks.

![image](https://github.com/user-attachments/assets/7a24e1e2-253c-49b0-a4d6-26de8af44202)

## Table of Contents

- [Project Overview](#project-overview)
- [Features](#features)
- [Hardware Components](#hardware-components)
- [Software Architecture](#software-architecture)
- [Getting Started](#getting-started)
- [Contributing](#contributing)
- [License](#license)
- [Acknowledgements](#acknowledgements)

## Project Overview

The Autonomous Robo Soccer Robot is engineered to switch seamlessly between manual and autonomous control modes. In manual mode, an operator can control the robot using a joystick interface. In autonomous mode, the robot utilizes computer vision to detect and track the soccer ball, navigate the field, avoid obstacles (including other robots), and attempt to score goals without human intervention.

## Features

- **Dual Control Modes**: Toggle between manual and autonomous operation.
- **Manual Control**: Utilize a joystick connected to an Arduino Nano via an HC11 module for directional movement and kicking.
- **Autonomous Control**: Employ an ESP32-CAM for ball recognition and tracking, with an STM32F103RB Nucleo board managing navigation and decision-making.
- **Obstacle Avoidance**: Detect and maneuver around other robots and obstacles on the field.
- **Goal Scoring**: Strategically approach and kick the ball towards the goal in autonomous mode.

## Hardware Components

- **Microcontrollers**:
  - *Arduino Nano*: Manages manual control inputs and communicates with the robot via HC11.
  - *STM32F103RB Nucleo Board*: Oversees autonomous operations, processing data from the ESP32-CAM and controlling movement.
- **Communication Modules**:
  - *HC11 Transceiver Modules*: Facilitate wireless communication between the joystick (manual control) and the robot.
- **Vision System**:
  - *ESP32-CAM*: Captures real-time video for ball detection and tracking during autonomous operation.
- **Control Interface**:
  - *Joystick with Push Button*: Provides manual control over movement (forward, reverse, left, right) and kicking mechanism.
- **Actuators**:
  - *Motors and Motor Drivers*: Enable precise movement and maneuverability.
  - *Kicking Mechanism*: Actuated to perform kicking actions during gameplay.

## Software Architecture

The software is structured to support both manual and autonomous functionalities:

- **Manual Mode**:
  - The joystick inputs are read by the Arduino Nano, which transmits commands via the HC11 module to the robot's HC11 receiver.
  - The STM32F103RB Nucleo board interprets these commands to control the motors and kicking mechanism accordingly.

- **Autonomous Mode**:
  - The ESP32-CAM captures video feed to detect and track the soccer ball using computer vision algorithms.
  - The STM32F103RB Nucleo board processes data from the ESP32-CAM to make navigation decisions, control the motors, avoid obstacles, and execute kicking actions when appropriate.

## Getting Started

To set up and operate the Autonomous Robo Soccer Robot:

1. **Assemble the Hardware**: Connect all components as per the hardware design specifications.
2. **Firmware Installation**:
   - Upload the manual control firmware to the Arduino Nano.
   - Upload the autonomous control firmware to the STM32F103RB Nucleo board.
   - Configure the ESP32-CAM with the computer vision software for ball detection.
3. **Operation**:
   - Use the mode switch to toggle between manual and autonomous modes.
   - In manual mode, control the robot using the joystick.
   - In autonomous mode, ensure the ESP32-CAM has a clear view for effective ball tracking and navigation.

## Contributing

We welcome contributions to enhance the capabilities of the Autonomous Robo Soccer Robot. Please adhere to the following guidelines:

- Fork the repository and create a new branch for your feature or bug fix.
- Ensure your code follows the project's coding standards.
- Test your changes thoroughly before submitting a pull request.
- Provide a clear description of your changes and the problem they solve.

## License

This project is licensed under the [MIT License](LICENSE), allowing for open collaboration and modification.

## Acknowledgements

This project builds upon concepts and technologies from various open-source initiatives and research in the field of robotic soccer. Notable references include:

- [Autonomous Soccer Playing Robot](https://www.instructables.com/Autonomous-Soccer-Playing-Robot/)
- [Ball Tracking ESP32 Robot Using OpenCV and Python](https://www.youtube.com/watch?v=W7tKPS5KX_o)
- [Fully Open Source Educational Nucleo-Based Robot](https://community.st.com/t5/stm32-mcus-products/fully-open-source-educational-nucleo-based-robot/td-p/461808)
- [CAD Design-0](https://www.tinkercad.com/things/53nqoq34h0A-robot-soccer)
- [CAD Design-1](https://grabcad.com/library/robo-soccer-bot-1)
- [CAD Design-2](https://grabcad.com/library/robosoccer-bot-basic-1)

For detailed tutorials and further reading, refer to the links provided above.

