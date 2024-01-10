# Ri3D-2024
Official code from UMN Robotics for the 2024 Ri3D/FRC season!
 
 Our robot subsystems include:
 1) **Drivetrain:**  We chose to use a tank drive this year, and our drivetrain commands include driving a specified distance, turning to a specific angle using a NavX gyroscope, and autonomously aiming at an Apriltag target and driving within range of it.
 2) **Intake:** The intake subsystem is responsible for collecting notes from the floor. We chose to go with an under-the-bumper intake powered by two NEO 550 motors, as it proved to be effective and also reduces the potential for penalties that over-the-bumper intakes have.
 3) **Launcher:** The launcher subsystem is responsible for shooting notes and is powered by a NEO motor. We use PID velocity control on the shooter.
 4) **Feeder:** The Feeder is the subsystem between the intake and the launcher that is responsible for storing notes and moving them into the launcher flywheel after it has gotten up to speed.
 5) **Climber:** Our climber is a modified (cut shorter) climber in the box from Andymark, powered by a 775pro motor that is heavily geared down.
 6) **Vision:** We are running a Photonvision pipeline on a Raspberry Pi 3 B+ to detect Apriltags, and we can track and follow either the nearest Apriltag or an Apriltag with a specific ID.
 7) **LED Subsystem:** We also wrote code for controlling RGB LED strips using a REV Blinkin LED driver to add some extra bling to our robot!
 8) **Power Subsystem:** This subsystem is for reading data from the REV PDH, such as the current draw of specific channels
 
How to get set up for FRC programming:
1) Install the latest release of WPILib and the latest release of the NI FRC Game tools [here](https://github.com/wpilibsuite/allwpilib/releases).
2) Use this open-source Ri3D repository as a template for your code if you'd like! :)

Our project is created using the "Timed Robot" template/style and our code is written in Java.
