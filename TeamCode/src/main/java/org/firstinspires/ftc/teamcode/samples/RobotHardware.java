package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftBack   = null;
    private DcMotor rightBack  = null;

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
        leftFront  = myOpMode.hardwareMap.get(DcMotor.class, "frontLeft");
        rightFront = myOpMode.hardwareMap.get(DcMotor.class, "frontRight");
        leftBack  = myOpMode.hardwareMap.get(DcMotor.class, "backLeft");
        rightBack = myOpMode.hardwareMap.get(DcMotor.class, "backRight");

        getLeftFront().setDirection(DcMotor.Direction.REVERSE);
        getRightFront().setDirection(DcMotor.Direction.FORWARD);
        getLeftBack().setDirection(DcMotor.Direction.REVERSE);
        getRightBack().setDirection(DcMotor.Direction.FORWARD);

        setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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

    public void setMode(DcMotor.RunMode mode) {
        getLeftFront().setMode(mode);
        getRightFront().setMode(mode);
        getLeftBack().setMode(mode);
        getRightBack().setMode(mode);
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

    public DcMotor getLeftFront() {
        return this.leftFront;
    }

    public DcMotor getRightFront() {
        return this.rightFront;
    }

    public DcMotor getLeftBack() {
        return this.leftBack;
    }

    public DcMotor getRightBack() {
        return this.rightBack;
    }
}
