package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Backstage Blue - Non Corner", group = "BackstageBlueAuton")
public class BackstageBlueNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;
    Utility.SpikeMark spikeMark;
    int aprilTagId;

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject();

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot, aprilTagId);

        if (targetFound) {
            placeSecondPixel();
            parkRobot();
        } else {
            targetNotFoundParkRobot();
        }
    }

    private void targetNotFoundParkRobot() {
        if (spikeMark == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  22);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  17);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  10);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  21);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  5);
        }
    }

    private void parkRobot() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  20);
        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  14);
        }    }

    private void placeSecondPixel() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  6.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  6.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.extendViperSlide(robot,true);
            Utility.panDeliveryAuton(robot);
            Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_STAGE);

            sleep(Constants.PAN_DOOR_AUTON_WAIT);
            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHomeAuton(robot);
            Utility.resetViperSlide(robot);
            Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  6.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.extendViperSlide(robot,true);
            Utility.panDeliveryAuton(robot);
            Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_STAGE);

            sleep(Constants.PAN_DOOR_AUTON_WAIT);
            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHomeAuton(robot);
            Utility.resetViperSlide(robot);
            Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  6.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.extendViperSlide(robot,true);
            Utility.panDeliveryAuton(robot);
            Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_STAGE);

            sleep(Constants.PAN_DOOR_AUTON_WAIT);
            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHomeAuton(robot);
            Utility.resetViperSlide(robot);
            Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);
        }    }

    private void moveToObject() {

        if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  23.0);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  9);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  12);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  15);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  4);
            Utility.turnToPID(robot, 90);

        } else if (spikeMark == Utility.SpikeMark.LEFT) {

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

    private int getAprilTagId(Utility.SpikeMark spikeMark) {
        switch (spikeMark) {
            case LEFT:
                return 1;
            case CENTER:
                return 2;
            case RIGHT:
                return 3;
        }
        return 0;
    }

    private Utility.SpikeMark getSpikeMark(FindRegionPipeline findRegionPipeline) {

        if ((findRegionPipeline.getLeftAvgFinal() - findRegionPipeline.getRightAvgFinal()) > Constants.REGION_AVG_FINAL_DIFFERENCE_THRESHOLD) {
            return Utility.SpikeMark.LEFT;
        } else if ((findRegionPipeline.getRightAvgFinal() - findRegionPipeline.getLeftAvgFinal()) > Constants.REGION_AVG_FINAL_DIFFERENCE_THRESHOLD) {
            return Utility.SpikeMark.CENTER;
        } else {
            return Utility.SpikeMark.RIGHT;
        }
    }
}