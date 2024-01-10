// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) 
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Physical Robot Constants //
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(6); // Convert from inches to meters
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER; // Measured in meters
	public static final double TRACK_WIDTH = Units.inchesToMeters(21.75); // Distance between centers of right and left wheels on robot (in meters)

    // Controller Input Axes //
    public static final int CONTROLLER_USB_PORT_ID = 0; // USB port that the controller is plugged in to
    public static final int RIGHT_VERTICAL_JOYSTICK_AXIS = 5;
    public static final int RIGHT_HORIZONTAL_JOYSTICK_AXIS = 4;
    public static final int LEFT_VERTICAL_JOYSTICK_AXIS = 1;
    public static final int LEFT_HORIZONTAL_JOYSTICK_AXIS = 0;
    public static final int X_BUTTON = 3;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int LEFT_TRIGGER_AXIS = 2;
    public static final int RIGHT_TRIGGER_AXIS = 3;
    public static final int PREV_BUTTON = 9;
    public static final int START_BUTTON = 10;

    // Talon SRX CAN IDs //
    // public static final int CLIMBER_MOTOR_ID = 6;

    // Victor PWM Ports //
    public static final int LEFT_FRONT_DRIVE_MOTOR_ID = 0;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_ID = 7;
    public static final int LEFT_REAR_DRIVE_MOTOR_ID = 8;
    public static final int RIGHT_REAR_DRIVE_MOTOR_ID = 2;

    public static final int CLIMBER_MOTOR_ID_DRIVETRAIN = 3;

    // Spark MAX CAN IDs //
    public static final int LOWER_INTAKE_BAR_MOTOR_ID = 5;
    public static final int UPPER_INTAKE_BAR_MOTOR_ID = 3;
    public static final int FLY_WHEEL_MOTOR_ID = 2;
    public static final int FEEDER_WHEEL_MOTOR_ID = 4;  

    // Other PWM Ports //
    public static final int LED_PWM_ID = 4;
    
    // DIO (Digital Input/Output) Channels //
    public static final int RIGHT_ENCODER_CHANNEL_A = 0;
    public static final int RIGHT_ENCODER_CHANNEL_B = 1;
    public static final int LEFT_ENCODER_CHANNEL_A = 2;
    public static final int LEFT_ENCODER_CHANNEL_B = 3;

    // Drivetrain Constants //
    public static final boolean DRIVE_INVERT_LEFT = false; // Whether to reverse the left drivetrain motors or not
    public static final boolean DRIVE_INVERT_RIGHT = true; // Whether to reverse the right drivetrain motors or not
    public static final double GYRO_TURN_KP = 0.007; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_ROATION_KP = 0.0175;
    public static final double TRACKED_TAG_DISTANCE_DRIVE_KP = 0.3; // P (Proportional) constant of a PID loop
    public static final double TRACKED_TAG_AREA_DRIVE_KP = 0.2; // P (Proportional) constant of a PID loop
    public static final double APRILTAG_POWER_CAP = 0.75;
    public static final double DRIVE_TURNING_THRESHOLD_DEGREES = 3;
    public static final int LEFT_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder
    public static final int RIGHT_ENCODER_COUNTS_PER_REV = 1440; // The number of encoder counts equal to one full revolution of the encoder 

    // Intake Constants //
    public static final boolean LOWER_INTAKE_BAR_INVERT = false;
    public static final double LOWER_INTAKE_BAR_SPEED = 0.8;
    public static final boolean UPPER_INTAKE_BAR_INVERT = true;
    public static final double UPPER_INTAKE_BAR_SPEED = 0.4;

    // Climber Constants //
    public static final boolean CLIMBER_INVERT = false;
    public static final double CLIMBER_SPEED = 0.6;
    // Launcher Constants //
    public static final boolean FLY_WHEEL_INVERT = false;
    public static final double FLY_WHEEL_HIGH_SPEED_RPM = 1000;
    public static final double FLY_WHEEL_HIGH_SPEED_POWER = 0.75;
    public static final double FLY_WHEEL_AMP_SPEED_RPM = -300;
    public static final double FLY_WHEEL_AMP_SPEED_POWER = 0.21;
    public static final boolean FEEDER_WHEEL_INVERT = true;
    public static final double FEEDER_WHEEL_SPEED = 0.15;

    public static final double FLY_WHEEL_P = 0.0001;
    public static final double FLY_WHEEL_I = 0.000001;
    public static final double FLY_WHEEL_D = 0.01;
    public static final double FLY_WHEEL_IZONE = 0;
    public static final double FLY_WHEEL_FF = 0.000173;
    public static final double FLY_WHEEL_K_MAX_OUTPUT = 1;
    public static final double FLY_WHEEL_K_MIN_OUTPUT = -1;
    public static final double FLY_WHEEL_MAX_RPM = 1000;
    public static final double FLY_WHEEL_MAX_POWER = 0.8;

    // REV PH Channels //
    public static final int EXTENSION_SOLENOID_ID = 1;

    // Rev PDH Constants //
    public static final int LEFT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 11;
    public static final int RIGHT_FRONT_DRIVE_MOTOR_PDH_CHANNEL = 10;
    public static final int LEFT_REAR_DRIVE_MOTOR_PDH_CHANNEL = 12;
    public static final int RIGHT_REAR_DRIVE_MOTOR_PDH_CHANNEL = 13;

    public static final int LOWER_INTAKE_MOTOR_PDH_CHANNEL = 14;
    public static final int UPPER_INTAKE_MOTOR_PDH_CHANNEL = 15;
    public static final int FLY_WHEEL_MOTOR_PDH_CHANNEL = 16;
    public static final int CLIMBER_MOTOR_PDH_CHANNEL = 17;
    public static final int FEEDER_WHEEL_MOTOR_PDH_CHANNEL = 18;

    // Apriltag Vision Constants //
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(7);
    public static final double TARGET_HEIGHT_METERS = Units.inchesToMeters(18.5);
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(18);
    public static final String USB_CAMERA_NAME = "USB_Camera-B4.09.24.1";
}