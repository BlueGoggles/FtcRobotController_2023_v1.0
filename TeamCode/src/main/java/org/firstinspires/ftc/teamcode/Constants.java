package org.firstinspires.ftc.teamcode;

public interface Constants {
    String DEVICE_FRONT_LEFT = "frontLeft";
    String DEVICE_FRONT_RIGHT = "frontRight";
    String DEVICE_BACK_LEFT = "backLeft";
    String DEVICE_BACK_RIGHT = "backRight";
    String DEVICE_INTAKE_WHEEL = "intakeWheel";
    String DEVICE_INTAKE_BELT = "intakeBelt";
    String DEVICE_VIPER_SLIDE = "viperSlide";
    String DEVICE_LEAD_SCREW = "leadScrew";
    String DEVICE_PAN_SERVO = "panServo";
    String DEVICE_PAN_DOOR = "panDoor";
    String DEVICE_LEAD_SCREW_SWITCH = "leadScrewSwitch";
    String DEVICE_DRONE_LAUNCHER = "droneLauncher";
    String DEVICE_IMU = "imu";
    String EXPANSION_IMU = "imu2";
    String DEVICE_CAMERA = "Camera1";
    double ZERO_POWER = 0.0;
    double MAX_POWER = 1.0;
    double AUTON_DRIVE_SPEED = 0.6;
    double AUTON_TURN_SPEED = 0.3;
    double OBJECT_WIDTH_IN_INCHES = 3.0;  // The actual width of the object in real-world units
    int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    int CAMERA_HEIGHT = 360; // height of wanted camera resolution
    double COUNTS_PER_MOTOR_REV = 537.7 ;    // eg: GoBILDA 312 RPM Yellow Jacket
    double DRIVE_GEAR_REDUCTION = 1.0 ;     // No External Gearing.
    double WHEEL_DIAMETER_INCHES = 3.78 ;     // For figuring out circumference
    double CAMERA_FOCAL_LENGTH = 793.33;
    double Kp = 0.01; // Proportional Gain
    double Ki = 0.0; // Integral Gain
    double Kd = 0.003; // Derivative Gain
    double DESIRED_DISTANCE_FROM_APRILTAG = 12.0; //  this is how close the camera should get to the target (inches)

    // Below are constants for moving Robot to the AprilTag.
    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    double REGION_AVG_FINAL_DIFFERENCE_THRESHOLD = 8.0;

    double MID_SERVO   =  0.5 ;

    int LEAD_SCREW_COUNT_UP = 13000;
    int LEAD_SCREW_COUNT_DOWN = 0;
    int VIPER_SLIDE_STAGE_1_COUNT = 1800;
    int VIPER_SLIDE_STAGE_2_COUNT = 2300;
    int VIPER_SLIDE_STAGE_3_COUNT = 2800;
    int VIPER_SLIDE_REST_COUNT = 0;
    int VIPER_SLIDE_VARIANCE = 100;

    double TELEOP_DEFAULT_SPEED = 0.5;
    double TELEOP_MODIFIED_SPEED = 1.0;

    double PAN_DOOR_START_POSITION = 0.0;
    double PAN_DOOR_STOP_POSITION = 0.5;

    int PAN_DOOR_RUN_TIME_PURPLE_PIXEL = 1500;
    int PAN_DOOR_RUN_TIME_YELLOW_PIXEL = 4000;

    int INITIAL_WAIT_TIME_FOR_FRONT_STAGE = 5000;

    // "10 / 8.5 = 1.1764".
    // Need to multiply by this ratio, because 10 inches of instructed strafe movement causes 8.5 inches of actual strafe movement.
    double STRAFE_MOVEMENT_RATIO = 1.1764;
}
