package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    private DcMotor leftFront   = null;
    private DcMotor rightFront  = null;
    private DcMotor leftBack   = null;
    private DcMotor rightBack  = null;

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setMotorPowers(0, 0,0,0);

        myOpMode.telemetry.addData("Initialization", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void setMotorPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        this.leftFront.setPower(leftFront);
        this.rightFront.setPower(rightFront);
        this.leftBack.setPower(leftBack);
        this.rightBack.setPower(rightBack);
    }
}
