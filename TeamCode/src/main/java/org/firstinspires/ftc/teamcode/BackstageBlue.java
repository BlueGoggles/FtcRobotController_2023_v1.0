package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Backstage Blue - Corner", group = "BackstageBlueAuton")
public class BackstageBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        blueAuton(robot, true);
    }

    public static void blueAuton(RobotHardware robot, boolean parkInCorner) {
        Utility.Color color = Utility.Color.BLUE;

        double distanceToPark = 14;

        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject(robot);

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot);

        if (targetFound) {
            Utility.placeSecondPixel(robot);
            if( parkInCorner ) {
                Utility.parkRobot(robot, color, distanceToPark);
            } else {
                Utility.parkRobotNonCorner(robot, color, ((2 * Constants.DISTANCE_BETWEEN_APRIL_TAGS) + distanceToPark) );
            }
        } else {
            targetNotFoundParkRobot(robot);
        }
    }

    public static void targetNotFoundParkRobot(RobotHardware robot) {
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  22);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  17);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  10);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  21);
            Utility.turnToPID(robot, Constants.HOME_DIRECTION);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  5);
        }
    }

    public static void moveToObject(RobotHardware robot) {

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  23.0);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  9);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  12);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  15);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  4);
            Utility.turnToPID(robot, 90);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  12.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  19.5);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  8);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  12);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  16);

        }
    }
}