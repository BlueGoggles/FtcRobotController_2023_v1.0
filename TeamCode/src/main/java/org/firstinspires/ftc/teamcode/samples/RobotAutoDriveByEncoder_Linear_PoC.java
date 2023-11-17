package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility;

@Autonomous(name="Robot: Auto Drive By Encoder PoC", group = "Samples")
@Disabled
public class RobotAutoDriveByEncoder_Linear_PoC extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.initialize();

        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at ",  "%7d :%7d :%7d :%7d",
                robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48,  48, 48,  48);  // S1: Forward 48 Inches
        encoderDrive(TURN_SPEED,   12, -12, 12, -12);  // S2: Turn Right 12 Inches
        encoderDrive(DRIVE_SPEED, -24, -24, -24, -24);  // S3: Reverse 24 Inches

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of two conditions occur:
     *  1) Move gets to the desired position
     *  2) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            robot.setTargetPosition(Utility.Direction.FORWARD, leftFrontInches, rightFrontInches, leftBackInches, rightBackInches);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Start the motion.
            robot.setMotorPowers(Math.abs(speed));

            while (opModeIsActive() && (robot.getLeftFront().isBusy() && robot.getRightFront().isBusy() && robot.getLeftBack().isBusy() && robot.getRightBack().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Currently at ",  "%7d :%7d :%7d :%7d",
                        robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setMotorPowers(0);

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }
}
