package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Frontstage Blue - Corner", group = "FrontstageBlueAuton")
public class FrontstageBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;

    @Override
    public void runOpMode() {

        boolean targetFound = false;

        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject(robot);

        // Move to desired AprilTag
        for (int counter = 0; counter < 3; counter++) {

            targetFound = Utility.moveToAprilTag(robot, Constants.BLUE_APRIL_TAG_ID);

            if (targetFound) {
                break;
            } else {
                if (counter == 0) {
                    Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, Constants.APRIL_TAG_NOT_FOUND_STRAFE_INCHES * Constants.STRAFE_MOVEMENT_RATIO);
                } else if (counter == 1) {
                    Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 2 * Constants.APRIL_TAG_NOT_FOUND_STRAFE_INCHES * Constants.STRAFE_MOVEMENT_RATIO);
                } else {
                    Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, Constants.APRIL_TAG_NOT_FOUND_STRAFE_INCHES * Constants.STRAFE_MOVEMENT_RATIO);
                }
            }
        }

        if (targetFound) {
            placeSecondPixel(robot);
            parkRobot();
        } else {
            targetNotFoundParkRobot();
        }
    }

    private void targetNotFoundParkRobot() {

        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  17);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  21 * Constants.STRAFE_MOVEMENT_RATIO);
    }

    private void parkRobot() {

        double inches;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            inches = 19;
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 15 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 15 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        }

        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  inches);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  13 * Constants.STRAFE_MOVEMENT_RATIO);
    }

    public static void placeSecondPixel(RobotHardware robot) {

        Utility.Direction direction;
        double inches;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            direction = Utility.Direction.LEFT;
            inches = (Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES + Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + Constants.GRACE_INCHES_FOR_SECOND_PIXEL_PLACEMENT) * Constants.STRAFE_MOVEMENT_RATIO;

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            direction = Utility.Direction.LEFT;
            inches = Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES * Constants.STRAFE_MOVEMENT_RATIO;

        } else {

            direction = Utility.Direction.RIGHT;
            inches = (Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES - Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + Constants.GRACE_INCHES_FOR_SECOND_PIXEL_PLACEMENT) * Constants.STRAFE_MOVEMENT_RATIO;

        }

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  8);
        Utility.encoderDrive(robot, direction, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, inches);

        Utility.extendViperSlide(robot,true);
        Utility.panDeliveryAuton(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_STAGE);

        robot.getMyOpMode().sleep(Constants.PAN_DOOR_AUTON_WAIT);
        Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3);
        Utility.panHomeAuton(robot);
        Utility.resetViperSlide(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);
    }

    public static void moveToObject(RobotHardware robot) {

        robot.getMyOpMode().sleep(Constants.INITIAL_WAIT_TIME_FOR_FRONT_STAGE);

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  27);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  73);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.BLUE_LEFT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  13 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  90);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.BLUE_LEFT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  73);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.BLUE_LEFT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);
        }
    }
}