package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "BackstageRed")
public class BackstageRed extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private double objectWidthInRealWorldUnits = 3.0;  // The actual width of the object in real-world units

    private ColorDetectionPipeline.Color color = ColorDetectionPipeline.Color.BLUE;

    static final double     DRIVE_SPEED             = 0.2;
    static final double     TURN_SPEED              = 0.3;

    @Override
    public void runOpMode() {

        ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline(objectWidthInRealWorldUnits, color);
        robot.initializeOpenCV(colorDetectionPipeline);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(robot.getControlHubCam(), 30);

        robot.initialize();

        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at ",  "%7d :%7d :%7d :%7d",
                robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
        telemetry.update();

        waitForStart();

//        while (opModeIsActive()) {
//            telemetry.addData("Coordinate", "(" + (int) colorDetectionPipeline.getcX() + ", " + (int) colorDetectionPipeline.getcY() + ")");
//            telemetry.addData("Distance in Inch", (colorDetectionPipeline.getDistance(colorDetectionPipeline.getWidth())));
//            telemetry.update();
//
//            // The OpenCV pipeline automatically processes frames and handles detection
//        }

        double distanceToMove = (colorDetectionPipeline.getDistance(colorDetectionPipeline.getWidth()) - 4.5);

        telemetry.addData("Distance to move : ", distanceToMove);
        telemetry.update();

        encoderDrive(DRIVE_SPEED,  distanceToMove,  distanceToMove, distanceToMove,  distanceToMove);
        // Release resources
        robot.getControlHubCam().stopStreaming();
    }

    public void encoderDrive(double speed, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            robot.setTargetPosition(leftFrontInches, rightFrontInches, leftBackInches, rightBackInches);

            // Turn On RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            // Start the motion.
            robot.setMotorPowers(Math.abs(speed));

            while (opModeIsActive() && (robot.getLeftFront().isBusy() && robot.getRightFront().isBusy() && robot.getLeftBack().isBusy() && robot.getRightBack().isBusy())) {

                // Display it for the driver.
                telemetry.addData("Currently at ",  "%7d :%7d :%7d :%7d",
                        robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition(), robot.getLeftFront().getCurrentPosition(), robot.getRightFront().getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.setMotorPowers(0);

            // Turn off RUN_TO_POSITION
            robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }
    }




}