package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Frontstage Red - Non Corner", group = "FrontstageRedAuton")
public class FrontstageRedNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.RED;

    @Override
    public void runOpMode() {

        boolean targetFound = false;

        Utility.initializeRobot(robot, color);

        // Drive towards object
        FrontstageRed.moveToObject(robot);

        // Move to desired AprilTag
        for (int counter = 0; counter < 3; counter++) {

            targetFound = Utility.moveToAprilTag(robot, Constants.RED_APRIL_TAG_ID);

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
            FrontstageRed.placeSecondPixel(robot);
            parkRobot();
        } else {
            targetNotFoundParkRobot();
        }
    }

    private void targetNotFoundParkRobot() {
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  13);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  24);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  12);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  12);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  10);
        }
    }

    private void parkRobot() {
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  1);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  14);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  20);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  26);
        }
    }
}