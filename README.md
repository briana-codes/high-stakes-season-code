# VEX Robotics High Stakes - Autonomous & TeleOp Control

A competitive VEX V5 robotics program developed for the 2024-2025 High Stakes season, featuring advanced autonomous routines and optimized teleoperated control systems.

## About High Stakes

High Stakes is the 2024-2025 VEX V5 Robotics Competition game played on a 12'x12' field. The objective is to score points by:
- Scoring rings on stakes (1 point each, 3 points for top ring)
- Placing mobile goals in positive or negative corners
- Climbing the central ladder at match end (3-12 points based on level)
- Winning the 15-second autonomous period (6 bonus points) 

## Features

### Autonomous Control
- **PID-based movement**: Precise driving, turning, and swing movements using tuned PID constants (kP=7.1, kI=2.0, kD=6.05) 
- **Strategic autonomous routine**: Automated sequence for scoring rings on stakes, manipulating mobile goals, and positioning for ladder climb 
- **Motion chaining**: Smooth blended movements for efficient autonomous execution 
- **Exit conditions**: Smart timeout and threshold-based movement completion 
### Teleoperated Control
- **Custom drive control**: Tank drive with reverse cubic scaling for smooth, responsive control 
- **Variable speed modes**: L1 button toggles 50% speed reduction for precise maneuvering
- **Advanced intake system**: Automated object detection using distance sensors with threshold-based stopping (120mm) 
- **Pneumatic control**: Toggle-based control for hook ("undita"), claw ("clema"), and palette mechanisms 
- **Catapult system**: Dual-mode operation with preset positions for consistent launches 

## Technical Stack

### Libraries & Frameworks
- **PROS (Purdue Robotics Operating System)**: Core development platform for VEX V5 
- **EZ-Template**: PROS library providing PID control, autonomous selector, and drive base management 

### Algorithms & Control Systems
- **PID Control**: Proportional-Integral-Derivative controllers for precise autonomous movement
- **Slew Rate Control**: Gradual acceleration for drive motions (7 inches slew distance)
- **Reverse Cubic Scaling**: Non-linear joystick response curve for enhanced driver control
- **State Machine Logic**: Toggle-based controls for all pneumatic and motor systems

### Hardware Integration
- **Motors**: Blue, green, and red gear cartridge configurations for optimal torque/speed ratios
- **Sensors**: IMU (port 14), rotation sensors for odometry (ports 12-13), distance sensors (ports 2, 6), and bump sensors
- **Pneumatics**: ADI digital outputs controlling hook, claw, palette, and activator mechanisms

## Key Autonomous Routine

The competition autonomous executes a 13-step sequence:
1. Drive backward 35 inches to approach stake
2. Turn 38° to align with target
3. Deploy hook mechanism to grab mobile goal
4. Engage claw to secure stake
5. Score pre-loaded ring using intake system
6. Navigate to second ring location
7. Intake additional ring while moving
8. Rotate 123° toward ladder zone
9. Drive backward 45 inches to final position [file:1]

## Constants & Tuning

### PID Constants
```cpp
// Heading: kP=11, kI=0, kD=20
// Drive: kP=20, kI=0, kD=100
// Turn: kP=3, kI=0.05, kD=20, startI=15
// Swing: kP=6, kI=0, kD=65
```

### Exit Conditions
- Turn/Swing: 80ms @ 3°, 250ms @ 7°, 500ms timeout [file:2]
- Drive: 80ms @ 1", 250ms @ 3", 500ms timeout [file:2]

## Installation

1. Install PROS for Visual Studio Code
2. Clone this repository
3. Open the project in VS Code
4. Build and upload to VEX V5 Brain

## License

Open source - developed for educational and competitive robotics purposes.
