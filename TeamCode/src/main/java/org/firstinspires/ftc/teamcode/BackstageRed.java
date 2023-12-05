package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Backstage Red - Corner", group = "BackstageRedAuton")
public class BackstageRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.RED;

    @Override
    public void runOpMode() {
        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject(robot);

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot);

        if (targetFound) {
            Utility.placeSecondPixel(robot);
            Utility.parkRobot(robot, color, 29);
        } else {
            targetNotFoundParkRobot(robot);
        }
    }

    public static void targetNotFoundParkRobot(RobotHardware robot) {
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  20);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  22);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  27);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  16);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  21);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  6);
        }
        Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
    }

    public static void moveToObject(RobotHardware robot) {

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  12);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  12);
            Utility.turnToPID(robot, -90);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  4 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  6);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  18);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  6);
            Utility.turnToPID(robot, -90);

        }
    }
}