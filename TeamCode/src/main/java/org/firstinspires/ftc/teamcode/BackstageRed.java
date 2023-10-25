package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BackstageRed")
public class BackstageRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private ColorDetectionPipeline.Color color = ColorDetectionPipeline.Color.BLUE;

    @Override
    public void runOpMode() {

        // Initialize Robot with Encoder
        robot.initialize();
        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize OpenCV
        ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline(Constants.OBJECT_WIDTH_IN_INCHES, color);
        robot.initializeOpenCV(colorDetectionPipeline);

        // Initialize Gyro sensor
        robot.initializeIMU();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at ",  "%7d :%7d :%7d :%7d",
                robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
        telemetry.update();

        waitForStart();

        // Get the distance of the object from camera. Reduce the distance by 5 inches to stop robot little earlier before reaching the object.
        double distanceToMove = (colorDetectionPipeline.getDistance(colorDetectionPipeline.getWidth()) - 5);

        telemetry.addData("Distance to move : ", distanceToMove);
        telemetry.update();

        // Release camera resources
        robot.getControlHubCam().stopStreaming();

        // Drive towards object
        Utility.encoderDrive(robot, Constants.AUTON_DRIVE_SPEED,  distanceToMove,  distanceToMove, distanceToMove,  distanceToMove);

        // Turn to absolute 90 degrees clockwise
        Utility.turnToPID(robot, -90);
    }
}