package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Frontstage Blue - Corner", group = "FrontstageBlueAuton")
public class FrontstageBlue extends LinearOpMode {

    protected RobotHardware robot = new RobotHardware(this);
    protected Utility.Color color = Utility.Color.BLUE;
    protected boolean targetFound = false;

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject(robot);

        robot.initializeAprilTag();
        Utility.setManualExposure(robot,Constants.CAMERA_EXPOSURE_MS, Constants.CAMERA_GAIN);  // Use low exposure time to reduce motion blur
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

        if ( ! targetFound) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  2);
        }

        placeSecondPixel(robot);
        parkRobot();
    }

    protected void parkRobot() {

        double inches;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            inches = 19;
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 17 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 18 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        }

        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  inches);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  13 * Constants.STRAFE_MOVEMENT_RATIO);
    }

    protected void placeSecondPixel(RobotHardware robot) {

        Utility.Direction direction;
        double inches;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            direction = Utility.Direction.LEFT;
            inches = (Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES + Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + ( targetFound ? Constants.GRACE_INCHES_FOR_SECOND_PIXEL_PLACEMENT : 0 )) * Constants.STRAFE_MOVEMENT_RATIO;

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            direction = Utility.Direction.LEFT;
            inches = Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES * Constants.STRAFE_MOVEMENT_RATIO;

        } else {

            direction = Utility.Direction.RIGHT;
            inches = (Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES - Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + ( targetFound ? Constants.GRACE_INCHES_FOR_SECOND_PIXEL_PLACEMENT : 0 )) * Constants.STRAFE_MOVEMENT_RATIO;

        }

        Utility.encoderDrive(robot, direction, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, inches);

        Utility.extendViperSlide(robot,true, Utility.StageLocations.FRONT);
        Utility.panDeliveryAuton(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_FRONT_STAGE);
        robot.getMyOpMode().sleep(300);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_NEAR_BOARD_SPEED,  8);

        robot.getMyOpMode().sleep(Constants.PAN_DOOR_AUTON_WAIT);
        Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_NEAR_BOARD_SPEED,  3);
        Utility.panHomeAuton(robot);
        Utility.resetViperSlide(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);
    }

    protected void moveToObject(RobotHardware robot) {

        robot.getMyOpMode().sleep(Constants.INITIAL_WAIT_TIME_FOR_FRONT_STAGE);

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  29);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  74);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.BLUE_LEFT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  13 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  91.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.BLUE_LEFT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  78);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.BLUE_LEFT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);
        }
    }
}