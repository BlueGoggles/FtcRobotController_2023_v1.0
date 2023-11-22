package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class Utility {

    public enum Direction {
        LEFT,
        RIGHT,
        FORWARD,
        BACKWARD
    }

    public enum Color {
        RED(1),
        BLUE(2);

        private final int code;

        Color(int code) {
            this.code = code;
        }

        public int getCode() {
            return this.code;
        }
    }

    public enum SpikeMark {
        LEFT,
        CENTER,
        RIGHT
    }

    public enum PanStates {
        HOME,
        DEPLOYED
    }

    // By default we want the position to be the HOME position.
    private static PanStates requestedPanState = PanStates.HOME;

    public static void encoderDrive(RobotHardware robot, Utility.Direction direction, double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {

        // Ensure that the OpMode is still active
        if (robot.getMyOpMode().opModeIsActive()) {

            // Set Target Position
            robot.setTargetPosition(direction, leftFrontInches, rightFrontInches, leftBackInches, rightBackInches);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Start the motion.
            robot.setMotorPowers(Math.abs(speed));

            while (robot.getMyOpMode().opModeIsActive() &&
                    (robot.getLeftFront().isBusy() && robot.getRightFront().isBusy() && robot.getLeftBack().isBusy() && robot.getRightBack().isBusy())) {
                // Engage the control
            }

            // Stop all motion;
            robot.setMotorPowers(Constants.ZERO_POWER);

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }

    public static void encoderDrive(RobotHardware robot, Utility.Direction direction, double speed, double inches) {
        encoderDrive(robot, direction, speed,  inches,  inches, inches,  inches);
    }

    public static void turnToPID(RobotHardware robot, double targetAngle) {

        PIDController pid = new PIDController(targetAngle, Constants.Kp, Constants.Ki, Constants.Kd);

        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (robot.getMyOpMode().opModeIsActive() && (Math.abs(targetAngle - robot.getAbsoluteAngle()) > 1 || pid.getLastSlope() > 0.75)) {

            double motorPower = pid.update(robot.getAbsoluteAngle());
            robot.setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);
        }

        robot.setMotorPowers(Constants.ZERO_POWER);
    }
    public static void turnPID(RobotHardware robot, double degrees) {
        turnToPID(robot,(degrees + robot.getAbsoluteAngle()));
    }

    public static boolean moveToAprilTag(RobotHardware robot, int desiredTagId) {

        AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        while (robot.getMyOpMode().opModeIsActive()) {
            targetFound = false;
            desiredTag  = null;

            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = robot.getAprilTag().getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if (detection.id == desiredTagId) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    }
                }
            }

            // Tell the driver what we see.
            if (targetFound) {
                robot.getMyOpMode().telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                robot.getMyOpMode().telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                robot.getMyOpMode().telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                robot.getMyOpMode().telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {
                robot.getMyOpMode().telemetry.addData("\n>","Target not found\n");
            }
            robot.getMyOpMode().telemetry.update();

            // If we have found the desired target, Drive to target Automatically.
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - Constants.DESIRED_DISTANCE_FROM_APRILTAG);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                if (Math.abs(rangeError) < 0.2) {
                    break;
                }
                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive  = Range.clip(rangeError * Constants.SPEED_GAIN, -Constants.MAX_AUTO_SPEED, Constants.MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * Constants.TURN_GAIN, -Constants.MAX_AUTO_TURN, Constants.MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * Constants.STRAFE_GAIN, -Constants.MAX_AUTO_STRAFE, Constants.MAX_AUTO_STRAFE);

                // Apply desired axes motions to the drivetrain.
                moveRobot(robot, drive, strafe, turn);
            }
        }
        return targetFound;
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    private static void moveRobot(RobotHardware robot, double x, double y, double yaw) {

        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        robot.setMotorPowers(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initializeAprilTag(), and only works for Webcams.
     Use low exposure time to reduce motion blur.
    */
    public static void setManualExposure(RobotHardware robot, int exposureMS, int gain) {

        // Wait for the camera to be open, then use the controls
        if (robot.getVisionPortal() == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (robot.getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING) {
            robot.getMyOpMode().telemetry.addData("Camera", "Waiting");
            robot.getMyOpMode().telemetry.update();
            while (!robot.getMyOpMode().isStopRequested() && (robot.getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING)) {
                robot.getMyOpMode().sleep(20);
            }
        }

        // Set camera controls unless we are stopping.
        if (!robot.getMyOpMode().isStopRequested()) {

            ExposureControl exposureControl = robot.getVisionPortal().getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                robot.getMyOpMode().sleep(50);
            }

            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            robot.getMyOpMode().sleep(20);

            GainControl gainControl = robot.getVisionPortal().getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            robot.getMyOpMode().sleep(20);
        }
    }

    public static void controlLeadScrew(RobotHardware robot, DcMotorSimple.Direction direction) {

        // Determine what position we need to go to.
        int leadScrewPosition = 0;
        if( direction == DcMotorSimple.Direction.FORWARD ) {
            leadScrewPosition = Constants.LEAD_SCREW_COUNT_UP;
        } else {
            leadScrewPosition = Constants.LEAD_SCREW_COUNT_DOWN;
        }

        robot.getLeadScrew().setDirection(direction);

        robot.getLeadScrew().setTargetPosition(leadScrewPosition);
        robot.getLeadScrew().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set the motor to full power for extend and retract.
        robot.getLeadScrew().setPower(Constants.MAX_POWER);
    }

    public static void extendViperSlide(RobotHardware robot) {
        robot.getViperSlide().setDirection(DcMotorEx.Direction.REVERSE);

        int slidePosition = robot.getViperSlide().getCurrentPosition();
        int stagePosition = Constants.VIPER_SLIDE_REST_COUNT;

        // First, check to see if the current position is below the lower end of the first stage 1.
        if( slidePosition <= ( Constants.VIPER_SLIDE_STAGE_1_COUNT - Constants.VIPER_SLIDE_VARIANCE ) ) {
            // In rest position. Go to stage 1.
            stagePosition = Constants.VIPER_SLIDE_STAGE_1_COUNT;
            // Next, check to see if the current position is between the upper and lower bounds of the first stage. If so, we are in the first stage, extend to the second.
        } else if ( ( slidePosition >= ( Constants.VIPER_SLIDE_STAGE_1_COUNT - Constants.VIPER_SLIDE_VARIANCE) ) &&
                ( slidePosition <= ( Constants.VIPER_SLIDE_STAGE_1_COUNT + Constants.VIPER_SLIDE_VARIANCE) ) ) {
            // In stage 1. Go to stage 2.
            stagePosition = Constants.VIPER_SLIDE_STAGE_2_COUNT;
            // We reach this case when the current position is greater than the upper limit of stage 2. We assume that we are in the second stage. So we extend to the third stage.
        } else {
            // In stage 2. Go to final stage, full extension.
            stagePosition = Constants.VIPER_SLIDE_STAGE_3_COUNT;
        }

        executeViperSlide( stagePosition, robot );
    }

    public static void retractViperSlide(RobotHardware robot) {
        robot.getViperSlide().setDirection(DcMotorEx.Direction.FORWARD);

        int slidePosition = robot.getViperSlide().getCurrentPosition();
        int stagePosition = Constants.VIPER_SLIDE_REST_COUNT;

        // First, check to see if the current position is less than the upper end of the last stage.
        if( slidePosition <= -( Constants.VIPER_SLIDE_STAGE_2_COUNT + Constants.VIPER_SLIDE_VARIANCE ) ) {
            // We are in the final stage, full extension. Move to stage 2.
            stagePosition = Constants.VIPER_SLIDE_STAGE_2_COUNT;
            // Next, check to see if the current position is between the upper and lower bounds of the second stage. If so, we are in the second stage, retract to the first.
        } else if ( ( slidePosition >= -( Constants.VIPER_SLIDE_STAGE_2_COUNT + Constants.VIPER_SLIDE_VARIANCE) ) &&
                ( slidePosition <= -( Constants.VIPER_SLIDE_STAGE_2_COUNT - Constants.VIPER_SLIDE_VARIANCE) ) ) {
            // In stage 2. Go to stage 1.
            stagePosition = Constants.VIPER_SLIDE_STAGE_1_COUNT;
            // We reach this case when the current position is less than the lower limit of stage 1. We assume that we are in the first stage. So we retract to the rest stage.
        } else {
            // In stage 1. Return to rest position.
            stagePosition = Constants.VIPER_SLIDE_REST_COUNT;
        }

        executeViperSlide( stagePosition, robot );
    }

    public static void resetViperSlide(RobotHardware robot) {
        robot.getViperSlide().setDirection(DcMotorEx.Direction.FORWARD);

        // Always go back to resting position.
        executeViperSlide( Constants.VIPER_SLIDE_REST_COUNT, robot );
    }

    public static void executeViperSlide( int stagePosition, RobotHardware robot ) {
        robot.getViperSlide().setTargetPosition(stagePosition);
        robot.getViperSlide().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.getViperSlide().setPower(Constants.MAX_POWER);
    }

    public static void checkSlideAndScrewMotors(RobotHardware robot) {
        if( robot.getMyOpMode().opModeIsActive() ) {
            // Check to see if the viper slide is moving. If not, stop it.
            if (robot.getViperSlide().isBusy()) {
                // Viper slide is moving! Don't do anything.
            } else {
                // Stop all motion;
                robot.getViperSlide().setPower(Constants.ZERO_POWER);

                // Turn off RUN_TO_POSITION
                robot.getViperSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Check to see if the lead screw is moving. If not, stop it.
            if (robot.getLeadScrew().isBusy()) {
                // Lead screw is moving! Don't do anything.
            } else {
                // Stop all motion;
                robot.getLeadScrew().setPower(Constants.ZERO_POWER);

                // Turn off RUN_TO_POSITION
                robot.getLeadScrew().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        } else {
            // Opmode is not running. Don't do anything.
        }
    }

    public static void checkPanPosition(RobotHardware robot ) {
        switch( requestedPanState ) {
            case HOME:
                if (robot.getPanServo().getPosition() < Constants.PAN_HOME_POSITION) {
                    robot.getPanServo().setPosition(robot.getPanServo().getPosition() + Constants.PAN_TILT_ANGLE);
                    // If this causes problems during run time we can remove this to avoid blocking the program for this period of time.
                    robot.getMyOpMode().sleep(Constants.PAN_TILT_TIME_MS);
                }
                break;
            case DEPLOYED:
                if (robot.getPanServo().getPosition() > Constants.PAN_DEPLOYED_POSITION) {
                    robot.getPanServo().setPosition(robot.getPanServo().getPosition() - Constants.PAN_TILT_ANGLE);
                    // If this causes problems during run time we can remove this to avoid blocking the program for this period of time.
                    robot.getMyOpMode().sleep(Constants.PAN_TILT_TIME_MS);
                }
                break;
            default:
                // Unhandled pan state. Don't do anything.
                break;
        }
    }
    public static void panHome(RobotHardware robot) {
        requestedPanState = PanStates.HOME;
//        while (robot.getPanServo().getPosition() < Constants.PAN_HOME_POSITION) {
//            robot.getPanServo().setPosition(robot.getPanServo().getPosition() + Constants.PAN_TILT_ANGLE);
//            robot.getMyOpMode().sleep(Constants.PAN_TILT_TIME_MS);
//        }
    }

    public static void panDelivery(RobotHardware robot) {
        requestedPanState = PanStates.DEPLOYED;
//        while (robot.getPanServo().getPosition() > Constants.PAN_DEPLOYED_POSITION) {
//            robot.getPanServo().setPosition(robot.getPanServo().getPosition() - Constants.PAN_TILT_ANGLE);
//            robot.getMyOpMode().sleep(Constants.PAN_TILT_TIME_MS);
//        }
    }

    public static void scrollPanDoor(RobotHardware robot, int milliSeconds) {
        robot.getPanDoor().setPosition(Constants.PAN_DOOR_START_POSITION);
        robot.getMyOpMode().sleep(milliSeconds);
        robot.getPanDoor().setPosition(Constants.PAN_DOOR_STOP_POSITION);
    }
}
