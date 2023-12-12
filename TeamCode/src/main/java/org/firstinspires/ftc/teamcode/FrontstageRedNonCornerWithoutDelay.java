package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Frontstage Red - Without Delay - Non Corner", group = "FrontstageRedAuton")
public class FrontstageRedNonCornerWithoutDelay extends FrontstageRed {

    protected void parkRobot() {

        double inches = 0;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            inches = 10;
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 10 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 10 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        }

        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  inches);
    }

    protected void moveToObject(RobotHardware robot) {

        robot.getMyOpMode().sleep(0);

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  11.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  27);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  76.5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.RED_RIGHT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  13 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  23);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  94.5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.RED_RIGHT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  22 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  78.5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  Constants.RED_RIGHT_STRAFING_FOR_APRIL_TAG * Constants.STRAFE_MOVEMENT_RATIO);
        }
    }
}