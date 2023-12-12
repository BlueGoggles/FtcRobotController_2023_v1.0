package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Frontstage Blue - ExtraOrdinary - Non Corner", group = "FrontstageBlueAuton")
public class FrontstageBlueExtraOrdinary extends FrontstageBlueNonCorner {

    protected void placeSecondPixel(RobotHardware robot) {

        Utility.Direction direction;
        double inches;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            direction = Utility.Direction.LEFT;
            inches = (Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES + Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + ( targetFound ? Constants.GRACE_INCHES_FOR_SECOND_PIXEL_PLACEMENT : 0 ) - 2) * Constants.STRAFE_MOVEMENT_RATIO;

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            direction = Utility.Direction.LEFT;
            inches = (Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + 2.0) * Constants.STRAFE_MOVEMENT_RATIO;

        } else {

            direction = Utility.Direction.RIGHT;
            inches = (Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES - Constants.MOVE_PAN_LEFT_IN_FRONT_OF_APRIL_TAG_INCHES + ( targetFound ? Constants.GRACE_INCHES_FOR_SECOND_PIXEL_PLACEMENT : 0 ) - 1.0) * Constants.STRAFE_MOVEMENT_RATIO;

        }

        Utility.encoderDrive(robot, direction, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, inches);

        Utility.extendViperSlide(robot,true, Utility.StageLocations.FRONT);
        Utility.panDeliveryAuton(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_FRONT_STAGE);
        robot.getMyOpMode().sleep(300);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_NEAR_BOARD_SPEED,  8);

        robot.getMyOpMode().sleep(Constants.PAN_DOOR_AUTON_WAIT);
        Utility.scrollPanDoor(robot, 500);

        robot.getMyOpMode().sleep(700);
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 1.5 * Constants.STRAFE_MOVEMENT_RATIO);
        } else {
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 1.5 * Constants.STRAFE_MOVEMENT_RATIO);
        }
        robot.getMyOpMode().sleep(700);

        Utility.scrollPanDoor(robot, 500);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_NEAR_BOARD_SPEED,  3);
        Utility.panHomeAuton(robot);
        Utility.resetViperSlide(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);
    }

    protected void moveToObject(RobotHardware robot) {

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  15);

            Utility.turnToPID(robot, -90);

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  4 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  20);

            Utility.turnToPID(robot, -90);

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  10);

            Utility.turnToPID(robot, -90);

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  7);
        }

        robot.getIntakeWheel().setPower(1);
        robot.getIntakeBelt().setPower(1);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  47.5);

//        robot.getIntakeWheel().setPower(0);
//        robot.getIntakeBelt().setPower(0);

        Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  29 * Constants.STRAFE_MOVEMENT_RATIO);

        Utility.turnToPID(robot, 90);
    }
}