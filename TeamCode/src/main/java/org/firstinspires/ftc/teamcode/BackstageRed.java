package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BackstageRed")
public class BackstageRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;
    Utility.SpikeMark spikeMark;
    int aprilTagId;

    @Override
    public void runOpMode() {

        // Initialize OpenCV
        FindRegionPipeline findRegionPipeline = new FindRegionPipeline(Utility.Color.BLUE);
        robot.initializeOpenCV(findRegionPipeline);
        sleep(5000);

        spikeMark = getSpikeMark(findRegionPipeline);
        aprilTagId = getAprilTagId(spikeMark);

        telemetry.addData("Left Average Final : ", findRegionPipeline.getLeftAvgFinal());
        telemetry.addData("Right Average Final : ", findRegionPipeline.getRightAvgFinal());
        telemetry.addData("Spike Mark : ", spikeMark);
        telemetry.addData("April Tag Id : ", aprilTagId);
        telemetry.update();

        // Release camera resources for OpenCV
        robot.releaseResourcesForOpenCV();

        // Initialize Robot with Encoder
        robot.initialize();
        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize Gyro sensor
        robot.initializeIMU();

        // Initialize the Apriltag Detection process
        robot.initializeAprilTag();

        // Wait for PLAY button to press
        waitForStart();

        // Drive towards object
        moveToObject();

        // Place Purple pixel on spike mark
        sleep(3000);

        // Move to desired AprilTag
        Utility.setManualExposure(robot,6, 250);  // Use low exposure time to reduce motion blur
        moveToAprilTag();
    }

    private void moveToAprilTag() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            // Turn to absolute 90 degrees clockwise
            Utility.turnToPID(robot, -90);
            Utility.moveToAprilTag(robot, aprilTagId);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            // Turn to absolute 90 degrees clockwise
            Utility.turnToPID(robot, -90);
            Utility.moveToAprilTag(robot, aprilTagId);
        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            // Turn to absolute 90 degrees clockwise
            Utility.turnToPID(robot, -90);
            Utility.moveToAprilTag(robot, aprilTagId);
        }
    }

    private void moveToObject() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  5,  5, 5,  5);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  20,  20, 20,  20);
            // Turn to absolute 90 degrees clockwise
            Utility.turnToPID(robot, 90);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25,  25, 25,  25);
        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10,  10, 10,  10);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  15,  15, 15,  15);
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