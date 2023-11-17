package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "FirstAuton", group = "Samples")
@Disabled
public class FirstAuton extends LinearOpMode {

    private DcMotor backLeft;
    private DcMotor frontLeft;
    private DcMotor backRight;
    private DcMotor frontRight;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");

        // Put initialization blocks here.
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if (opModeIsActive()) {
            // move forward
            backLeft.setPower(0.3);
            frontLeft.setPower(0.3);
            backRight.setPower(0.3);
            frontRight.setPower(0.3);
            sleep(2000);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
            // Turn right
            backLeft.setPower(0.3);
            frontLeft.setPower(0.3);
            backRight.setPower(-0.3);
            frontRight.setPower(-0.3);
            sleep(800);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
            // move forward
            backLeft.setPower(0.3);
            frontLeft.setPower(0.3);
            backRight.setPower(0.3);
            frontRight.setPower(0.3);
            sleep(1000);
            backLeft.setPower(0);
            frontLeft.setPower(0);
            backRight.setPower(0);
            frontRight.setPower(0);
//            // turn right
//            backLeft.setPower(0.5);
//            frontLeft.setPower(0.5);
//            backRight.setPower(-0.5);
//            frontRight.setPower(-0.5);
//            sleep(800);
//            backLeft.setPower(0);
//            frontLeft.setPower(0);
//            backRight.setPower(0);
//            frontRight.setPower(0);
//            // move forward
//            backLeft.setPower(0.5);
//            frontLeft.setPower(0.5);
//            backRight.setPower(0.5);
//            frontRight.setPower(0.5);
//            sleep(2000);
//            backLeft.setPower(0);
//            frontLeft.setPower(0);
//            backRight.setPower(0);
//            frontRight.setPower(0);
//            // turn right
//            backLeft.setPower(0.5);
//            frontLeft.setPower(0.5);
//            backRight.setPower(-0.5);
//            frontRight.setPower(-0.5);
//            sleep(800);
//            backLeft.setPower(0);
//            frontLeft.setPower(0);
//            backRight.setPower(0);
//            frontRight.setPower(0);
//            // move forward
//            backLeft.setPower(0.5);
//            frontLeft.setPower(0.5);
//            backRight.setPower(0.5);
//            frontRight.setPower(0.5);
//            sleep(2000);
//            backLeft.setPower(0);
//            frontLeft.setPower(0);
//            backRight.setPower(0);
//            frontRight.setPower(0);
//            // turn right
//            backLeft.setPower(0.5);
//            frontLeft.setPower(0.5);
//            backRight.setPower(-0.5);
//            frontRight.setPower(-0.5);
//            sleep(700);
//            backLeft.setPower(0);
//            frontLeft.setPower(0);
//            backRight.setPower(0);
//            frontRight.setPower(0);
        }
    }
}
