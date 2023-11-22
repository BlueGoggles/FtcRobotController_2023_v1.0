package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Backstage Red - Non Corner", group = "BackstageRedAuton")
public class BackstageRedNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.RED;
    Utility.SpikeMark spikeMark;
    int aprilTagId;

    @Override
    public void runOpMode() {

        // Initialize Robot with Encoder
        robot.initialize();
        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize Gyro sensor
        robot.initializeIMU();

        // Initialize OpenCV
        FindRegionPipeline findRegionPipeline = new FindRegionPipeline(color);
        robot.initializeOpenCV(findRegionPipeline);
        sleep(5000);

        while (opModeInInit()) {
            spikeMark = getSpikeMark(findRegionPipeline);
            aprilTagId = getAprilTagId(spikeMark);

            telemetry.addData("Left Average Final : ", findRegionPipeline.getLeftAvgFinal());
            telemetry.addData("Right Average Final : ", findRegionPipeline.getRightAvgFinal());
            telemetry.addData("Spike Mark : ", spikeMark);
            telemetry.addData("April Tag Id : ", aprilTagId);
            telemetry.update();
        }

        // Release camera resources for OpenCV
        robot.releaseResourcesForOpenCV();

        // Initialize the Apriltag Detection process
        robot.initializeAprilTag();

        // Drive towards object
        moveToObject();

        // Move to desired AprilTag
        Utility.setManualExposure(robot,6, 250);  // Use low exposure time to reduce motion blur
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

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  20);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  24);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  27);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  22);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  21);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  6);
        }
    }

    private void parkRobot() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  14);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  20);
        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
        }
    }

    private void placeSecondPixel() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  8 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHome(robot);
            Utility.resetViperSlide(robot);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHome(robot);
            Utility.resetViperSlide(robot);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * Constants.STRAFE_MOVEMENT_RATIO);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_YELLOW_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHome(robot);
            Utility.resetViperSlide(robot);
        }
    }

    private void moveToObject() {

        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  12);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  12);
            Utility.turnToPID(robot, -90);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  2 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  28);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  6);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10 * Constants.STRAFE_MOVEMENT_RATIO);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  22);

            Utility.scrollPanDoor(robot, Constants.PAN_DOOR_RUN_TIME_PURPLE_PIXEL);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, -90);
        }
    }

    private int getAprilTagId(Utility.SpikeMark spikeMark) {
        switch (spikeMark) {
            case LEFT:
                return 4;
            case CENTER:
                return 5;
            case RIGHT:
                return 6;
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