package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    private DcMotorEx leftFront   = null;
    private DcMotorEx rightFront  = null;
    private DcMotorEx leftBack   = null;
    private DcMotorEx rightBack  = null;

    private IMU imu = null;
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring out circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void initialize()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFront  = myOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, "frontRight");
        leftBack  = myOpMode.hardwareMap.get(DcMotorEx.class, "backLeft");
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, "backRight");

        getLeftFront().setDirection(DcMotorEx.Direction.REVERSE);
        getRightFront().setDirection(DcMotorEx.Direction.FORWARD);
        getLeftBack().setDirection(DcMotorEx.Direction.REVERSE);
        getRightBack().setDirection(DcMotorEx.Direction.FORWARD);

        setZeroPowerBehavior();

        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        setMotorPowers(0);

        myOpMode.telemetry.addData("Initialization", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void setMotorPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        this.getLeftFront().setPower(leftFront);
        this.getRightFront().setPower(rightFront);
        this.getLeftBack().setPower(leftBack);
        this.getRightBack().setPower(rightBack);
    }

    public void setMotorPowers(double allWhealPower) {
        setMotorPowers(allWhealPower, allWhealPower, allWhealPower, allWhealPower);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        getLeftFront().setMode(mode);
        getRightFront().setMode(mode);
        getLeftBack().setMode(mode);
        getRightBack().setMode(mode);
    }

    public void setZeroPowerBehavior() {
        getLeftFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getLeftBack().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightBack().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {

        // Determine new target position, and pass to motor controller
        int leftFrontTarget = getLeftFront().getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
        int rightFrontTarget = getRightFront().getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
        int leftBackTarget = getLeftBack().getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
        int rightBackTarget = getRightBack().getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

        // Set the Target Position
        getLeftFront().setTargetPosition(leftFrontTarget);
        getRightFront().setTargetPosition(rightFrontTarget);
        getLeftBack().setTargetPosition(leftBackTarget);
        getRightBack().setTargetPosition(rightBackTarget);
    }

    public void initializeIMU() {
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);
    }

    public DcMotorEx getLeftFront() {
        return this.leftFront;
    }

    public DcMotorEx getRightFront() {
        return this.rightFront;
    }

    public DcMotorEx getLeftBack() {
        return this.leftBack;
    }

    public DcMotorEx getRightBack() {
        return this.rightBack;
    }
}
