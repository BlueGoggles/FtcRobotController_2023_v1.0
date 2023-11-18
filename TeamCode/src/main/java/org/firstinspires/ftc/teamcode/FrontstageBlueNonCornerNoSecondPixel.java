package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Frontstage Blue - Non Corner - No Second Pixel", group = "FrontstageBlueAuton")
@Disabled
public class FrontstageBlueNonCornerNoSecondPixel extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;
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

    private void targetNotFoundParkRobot() {
        if (spikeMark == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  10);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  22);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  5);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  26);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  9);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  35);
        }
    }

    private void parkRobot() {
        if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  1);
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  14);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  20);
        } else if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
        }
    }

    private void placeSecondPixel() {
        if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5);
            double x = 1.1764;
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * x);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            robot.getPanDoor().setPosition(0.0);
            sleep(4000);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHome(robot);
            Utility.resetViperSlide(robot);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5);
            double x = 1.1764;
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * x);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            robot.getPanDoor().setPosition(0.0);
            sleep(4000);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHome(robot);
            Utility.resetViperSlide(robot);

        } else if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5);
            double x = 1.1764;
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  6.5 * x);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            robot.getPanDoor().setPosition(0.0);
            sleep(4000);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.panHome(robot);
            Utility.resetViperSlide(robot);
        }
    }

    private void moveToObject() {
        double x = 1.1764;
        if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  23);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10.5 * x);

            robot.getPanDoor().setPosition(0.0);
            sleep(1200);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  11 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  27);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  70);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  13 * x);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  3 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26.5);

            robot.getPanDoor().setPosition(0.0);
            sleep(1200);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  13 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  25);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  90);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  21 * x);

        } else if (spikeMark == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  2 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  26);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  5);

            robot.getPanDoor().setPosition(0.0);
            sleep(1200);
            robot.getPanDoor().setPosition(0.5);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  5);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  23 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  73);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  27 * x);
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