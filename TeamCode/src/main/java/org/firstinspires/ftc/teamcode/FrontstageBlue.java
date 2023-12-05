package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Frontstage Blue - Corner", group = "FrontstageBlueAuton")
public class FrontstageBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;

    @Override
    public void runOpMode() {
        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject(robot);

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot);

        if (targetFound) {
            Utility.placeSecondPixel(robot);
            Utility.parkRobot(robot, color, 18);
        } else {
            targetNotFoundParkRobot(robot);
        }
    }

    public static void targetNotFoundParkRobot(RobotHardware robot) {
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 10);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 20);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 5);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 26);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 9);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 35);
        }
    }

    public static void moveToObject( RobotHardware robot ) {

        robot.getMyOpMode().sleep(Constants.INITIAL_WAIT_TIME_FOR_FRONT_STAGE);
        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 23);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 10.5 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 2);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 28);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 70);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 7 * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 3 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 26.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 2);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 13 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 25);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 90);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 13 * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 25 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 73);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, 19 * Constants.STRAFE_MOVEMENT_RATIO);

        }
    }
}