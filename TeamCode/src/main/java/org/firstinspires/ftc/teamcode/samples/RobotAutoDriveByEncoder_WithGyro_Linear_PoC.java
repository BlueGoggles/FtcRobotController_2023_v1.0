package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility;

@Autonomous(name="Robot: Auto Drive By Encoder With Gyro PoC", group = "Samples")
@Disabled
public class RobotAutoDriveByEncoder_WithGyro_Linear_PoC extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        robot.initialize();
        robot.initializeIMU();

        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        robot.getImu().resetYaw();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at ",  "%7d :%7d :%7d :%7d",
                robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  48);  // S1: Forward 48 Inches
        encoderDrive(DRIVE_SPEED, -24);  // S3: Reverse 24 Inches

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
    public void encoderDrive(double speed, double inches) {

        double turnSpeed = 0.0;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            robot.setTargetPosition(Utility.Direction.FORWARD, inches);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            PIDController pid = new PIDController(0.0, 0.01, 0, 0.003);
            telemetry.setMsTransmissionInterval(50);

            // Start the motion.
            moveRobot(Math.abs(speed), 0);

            while (opModeIsActive() && (robot.getLeftFront().isBusy() && robot.getRightFront().isBusy() && robot.getLeftBack().isBusy() && robot.getRightBack().isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = pid.update(robot.getCurrentHeading());;

                // if driving in reverse, the motor correction also needs to be reversed
                if (inches < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(Math.abs(speed), turnSpeed);

                // Display it for the driver.
                telemetry.addData("Currently at ",  "%7d :%7d :%7d :%7d",
                        robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            moveRobot(0, 0);

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public void moveRobot(double drive, double turn) {

        double leftSpeed  = drive - turn;
        double rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0) {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        robot.setMotorPowers(leftSpeed, rightSpeed, leftSpeed, rightSpeed);
    }
}
