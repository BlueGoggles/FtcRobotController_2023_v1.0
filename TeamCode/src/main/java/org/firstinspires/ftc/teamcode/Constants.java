package org.firstinspires.ftc.teamcode;

public interface Constants {
    String DEVICE_FRONT_LEFT = "frontLeft";
    String DEVICE_FRONT_RIGHT = "frontRight";
    String DEVICE_BACK_LEFT = "backLeft";
    String DEVICE_BACK_RIGHT = "backRight";
    String DEVICE_IMU = "imu";
    String DEVICE_CAMERA = "Camera1";
    double ZERO_POWER = 0.0;
    double MAX_POWER = 1.0;
    double AUTON_DRIVE_SPEED = 0.5;
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
}
