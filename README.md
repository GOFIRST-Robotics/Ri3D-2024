# Ri3D-2024
Official code from UMN Robotics for the 2024 Ri3D/FRC season!
 
 Our subsystems include:
 1) **Drivetrain:**  We chose to use a tank drive this year, and our drivetrain commands include driving a specified distance, turning to a specified angle using a NavX gyroscope, and autonomously aiming at an Apriltag target and driving within range of it.
 2) **Intake:** The intake subsystem is responsible for collecting notes. We chose to go with an under-the-bumper intake powered by NEO 550 motors.
 3) **Launcher:** The launcher subsystem is responsible for shooting notes and is powered by a NEO motor.
 4) **Vision:** We are running a Photonvision pipeline on a Raspberry Pi 3 B+ to detect Apriltags, and we can track and follow either the nearest Apriltag or an Apriltag with a specific ID.
 5) **LED Subsystem:** We also wrote code for controlling RGB LED strips via a REV Blinkin module to add some extra bling to our robot!
 
How to get set up for FRC programming:
1) Install the latest release of WPILib [here](https://github.com/wpilibsuite/allwpilib/releases)
2) Install the latest NI FRC Game Tools [here](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)
3) Use this open-source repository as a template for your code if you'd like :)

Our project is created using the "Timed Robot" template/style with our code being written in Java.
