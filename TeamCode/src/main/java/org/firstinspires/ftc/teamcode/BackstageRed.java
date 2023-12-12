package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Backstage Red - Corner", group = "BackstageRedAuton")
public class BackstageRed extends LinearOpMode {

    protected RobotHardware robot = new RobotHardware(this);
    protected Utility.Color color = Utility.Color.RED;
    protected ElapsedTime playtime = new ElapsedTime();
    protected boolean targetFound = false;

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot, color);
        playtime.reset();

        // Drive towards object
        moveToObject(robot);

        // Initialize the Apriltag Detection process
        robot.initializeAprilTag();
        Utility.setManualExposure(robot,Constants.CAMERA_EXPOSURE_MS, Constants.CAMERA_GAIN);  // Use low exposure time to reduce motion blur

        // Move to desired AprilTag
        for (int counter = 0; counter < 3; counter++) {

            targetFound = Utility.moveToAprilTag(robot, Constants.RED_APRIL_TAG_ID);

            if (targetFound) {
                break;
            } else {
                if (counter == 0) {
                    Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED, Constants.APRIL_TAG_NOT_FOUND_STRAFE_INCHES * Constants.STRAFE_MOVEMENT_RATIO);
                } else if (counter == 1) {
                    Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED, 2 * Constants.APRIL_TAG_NOT_FOUND_STRAFE_INCHES * Constants.STRAFE_MOVEMENT_RATIO);
                } else {
                    Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED, Constants.APRIL_TAG_NOT_FOUND_STRAFE_INCHES * Constants.STRAFE_MOVEMENT_RATIO);
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
        double parkingStrafeDistance = 10;

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            inches = 16;
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 17 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 16 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        }

        Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED, (inches - 4) * Constants.STRAFE_MOVEMENT_RATIO);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  8);

        if (Constants.WAIT_TIME_FOR_BACKSTAGE_PARKING_BEFORE_AUTON_ENDS > 0) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  25 * Constants.STRAFE_MOVEMENT_RATIO);

            while (playtime.milliseconds() < (Constants.AUTON_PLAY_TIME - Constants.WAIT_TIME_FOR_BACKSTAGE_PARKING_BEFORE_AUTON_ENDS)) {
                // Engage the control until wait time is over for backstage parking.
            }

            parkingStrafeDistance = parkingStrafeDistance + 25;
        }

        Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  parkingStrafeDistance * Constants.STRAFE_MOVEMENT_RATIO);
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

        Utility.encoderDrive(robot, direction, Constants.AUTON_DRIVE_SPEED, inches);

        Utility.extendViperSlide(robot,true, Utility.StageLocations.BACK);
        Utility.panDeliveryAuton(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.AUTON_BACK_STAGE);
        robot.getMyOpMode().sleep(300);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_NEAR_BOARD_SPEED,  8.5);

        robot.getMyOpMode().sleep(Constants.PAN_DOOR_AUTON_WAIT);
        Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_NEAR_BOARD_SPEED,  3);
        Utility.panHomeAuton(robot);
        Utility.resetViperSlide(robot);
        Utility.overrideViperSlideState(Utility.ViperSlideStates.HOME);
    }

    protected void moveToObject(RobotHardware robot) {

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  23.0);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  10.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  26.5);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  4 * Constants.STRAFE_MOVEMENT_RATIO);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25.5);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  24 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  0.5);
            Utility.turnToPID(robot, -90);

        } else if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  11.5 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  19.5);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  8);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  16);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  8 * Constants.STRAFE_MOVEMENT_RATIO);
        }
    }
}