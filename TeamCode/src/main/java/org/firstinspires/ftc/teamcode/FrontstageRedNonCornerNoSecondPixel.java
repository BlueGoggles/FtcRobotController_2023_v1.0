package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Frontstage Red - Non Corner - No Second Pixel", group = "FrontstageRedAuton")
@Disabled
public class FrontstageRedNonCornerNoSecondPixel extends LinearOpMode {

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
    }

    private void moveToObject() {
        double x = 1.1764;
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  23);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  10.5 * x);

            robot.getPanDoor().setPosition(0.0);
            sleep(1200);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  11 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  27);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  80);
            Utility.turnToPID(robot, 0);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  3 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26.5);

            robot.getPanDoor().setPosition(0.0);
            sleep(1200);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  13 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  98);
            Utility.turnToPID(robot, 0);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  2 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  5);

            robot.getPanDoor().setPosition(0.0);
            sleep(1200);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  5);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  23 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  81);
            Utility.turnToPID(robot, 0);
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