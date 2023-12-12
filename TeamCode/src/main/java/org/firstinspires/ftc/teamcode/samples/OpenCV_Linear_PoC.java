package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ColorDetectionPipeline;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility;

@Autonomous(name = "OpenCV Linear PoC", group = "Samples")
@Disabled
public class OpenCV_Linear_PoC extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private double objectWidthInRealWorldUnits = 3.0;  // The actual width of the object in real-world units

    private Utility.Color color = Utility.Color.BLUE;

    @Override
    public void runOpMode() {

        ColorDetectionPipeline colorDetectionPipeline = new ColorDetectionPipeline(objectWidthInRealWorldUnits, color);
        robot.initializeOpenCV(colorDetectionPipeline);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(robot.getControlHubCam(), 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) colorDetectionPipeline.getcX() + ", " + (int) colorDetectionPipeline.getcY() + ")");
            telemetry.addData("Distance in Inch", (colorDetectionPipeline.getDistance(colorDetectionPipeline.getWidth())));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        robot.getControlHubCam().stopStreaming();
    }
}