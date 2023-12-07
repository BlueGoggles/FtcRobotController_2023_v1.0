package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Backstage Blue - Corner", group = "BackstageBlueAuton")
public class BackstageBlue extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;

    @Override
    public void runOpMode() {

        boolean targetFound = false;

        Utility.initializeRobot(robot, color);

        // Drive towards object
        moveToObject(robot);

        // Initialize the Apriltag Detection process
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

        if (targetFound) {
            placeSecondPixel(robot);
            parkRobot();
        } else {
            targetNotFoundParkRobot();
        }
    }

    private void targetNotFoundParkRobot() {
        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  22);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  17);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  10);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  21);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  5);
        }
    }

    private void parkRobot() {

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

        Utility.encoderDrive(robot, direction, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED, inches);

        Utility.extendViperSlide(robot,true);
        Utility.panDeliveryAuton(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_STAGE);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  8.5);

        robot.getMyOpMode().sleep(Constants.PAN_DOOR_AUTON_WAIT);
        Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  3);
        Utility.panHomeAuton(robot);
        Utility.resetViperSlide(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);
    }

    public static void moveToObject(RobotHardware robot) {

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  23.0);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  10.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  28.5);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  7);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  26 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8);
            Utility.turnToPID(robot, 90);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  11.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  19.5);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  8);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  18);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  20 * Constants.STRAFE_MOVEMENT_RATIO);
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