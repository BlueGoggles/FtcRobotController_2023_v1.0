package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BackstageBlue", group = "BlueAuton")
public class BackstageBlue extends LinearOpMode {

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

        // Place Purple pixel on spike mark
//        sleep(3000);

        // Move to desired AprilTag
        Utility.setManualExposure(robot,6, 250);  // Use low exposure time to reduce motion blur
        moveToAprilTag();

        placeSecondPixel();

        parkRobot();
    }

    private void parkRobot() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  14,  14, 14,  14);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  18,  18, 18,  18);
        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.turnToPID(robot, 0);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  28,  28, 28,  28);
        }
    }

    private void placeSecondPixel() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5,  8.5, 8.5,  8.5);
            double x = 1.1764;
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  6.5 * x,  6.5 * x, 6.5 * x,  6.5 * x);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            robot.getPanDoor().setPosition(0.0);
            sleep(4000);
            robot.getPanDoor().setPosition(0.5);

            Utility.panHome(robot);
            Utility.resetViperSlide(robot);
            while( robot.getViperSlide().isBusy() ) {
                // Wait for the Viper Slide to retract.
            }
//            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4,  4, 4,  4);

        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5,  8.5, 8.5,  8.5);
            double x = 1.1764;
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * x,  5 * x, 5 * x,  5 * x);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            robot.getPanDoor().setPosition(0.0);
            sleep(4000);
            robot.getPanDoor().setPosition(0.5);

            Utility.panHome(robot);
            Utility.resetViperSlide(robot);
            while( robot.getViperSlide().isBusy() ) {
                // Wait for the Viper Slide to retract.
            }
//            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4,  4, 4,  4);

        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  8.5,  8.5, 8.5,  8.5);
            double x = 1.1764;
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  5 * x,  5 * x, 5 * x,  5 * x);

            Utility.extendViperSlide(robot);
            Utility.panDelivery(robot);

            robot.getPanDoor().setPosition(0.0);
            sleep(4000);
            robot.getPanDoor().setPosition(0.5);

            Utility.panHome(robot);
            Utility.resetViperSlide(robot);
            while( robot.getViperSlide().isBusy() ) {
                // Wait for the Viper Slide to retract.
            }
//            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4,  4, 4,  4);

        }
    }

    private void moveToAprilTag() {
        if (spikeMark == Utility.SpikeMark.LEFT) {
            // Turn to absolute 90 degrees clockwise
//            Utility.turnToPID(robot, -90);
            Utility.moveToAprilTag(robot, aprilTagId);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            // Turn to absolute 90 degrees clockwise
//            Utility.turnToPID(robot, -90);
            Utility.moveToAprilTag(robot, aprilTagId);
        } else if (spikeMark == Utility.SpikeMark.RIGHT) {
            // Turn to absolute 90 degrees clockwise
//            Utility.turnToPID(robot, -90);
            Utility.moveToAprilTag(robot, aprilTagId);
        }
    }

    private void moveToObject() {
        double x = 1.1764;
        if (spikeMark == Utility.SpikeMark.RIGHT) {
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  10 * x,  10 * x, 10 * x,  10 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  27,  27, 27,  27);
//            // Turn to absolute -90 degrees clockwise
            Utility.turnToPID(robot, -90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  12,  12, 12,  12);
            robot.getPanDoor().setPosition(0.0);
            sleep(1500);
            robot.getPanDoor().setPosition(0.5);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  12,  12, 12,  12);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  10,  10, 10,  10);
        } else if (spikeMark == Utility.SpikeMark.CENTER) {
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  2 * x,  2 * x, 2 * x,  2 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  28,  28, 28,  28);
//            // Turn to absolute 90 degrees clockwise
//            Utility.turnToPID(robot, 90);
//            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  3,  3, 3,  3);
            robot.getPanDoor().setPosition(0.0);
            sleep(1500);
            robot.getPanDoor().setPosition(0.5);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  4,  4, 4,  4);
            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  15,  15, 15,  15);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  4,  4, 4,  4);
            Utility.turnToPID(robot, 90);
//            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  12 * x,  12 * x, 12 * x,  12 * x);
        } else if (spikeMark == Utility.SpikeMark.LEFT) {

            Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED,  10 * x,  10 * x, 10 * x,  10 * x);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  22,  22, 22,  22);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  2,  2, 2,  2);
//            // Turn to absolute 90 degrees clockwise
//            Utility.turnToPID(robot, 90);
//            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  3,  3, 3,  3);
            robot.getPanDoor().setPosition(0.0);
            sleep(1500);
            robot.getPanDoor().setPosition(0.5);
            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED,  8,  8, 8,  8);
            Utility.turnToPID(robot, 90);
            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  12,  12, 12,  12);
            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  12,  12, 12,  12);
//            Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED,  12 * x,  12 * x, 12 * x,  12 * x);
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