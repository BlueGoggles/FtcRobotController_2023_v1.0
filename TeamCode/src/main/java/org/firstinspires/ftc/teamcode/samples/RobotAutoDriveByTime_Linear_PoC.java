package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name="Robot: Auto Drive By Time", group = "Samples")
@Disabled
public class RobotAutoDriveByTime_Linear_PoC extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private ElapsedTime  runtime = new ElapsedTime();

    static final double     FORWARD_SPEED = 0.5;
    static final double     TURN_SPEED    = 0.4;

    @Override
    public void runOpMode() {

        robot.initialize();

        telemetry.addData("Status", "Ready to run the ROBOT");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        robot.setMotorPowers(FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            telemetry.addData("Moving Forward", "");
            telemetry.update();
        }
        robot.setMotorPowers(0, 0,0,0);
        sleep(1000);

        robot.setMotorPowers(-TURN_SPEED, TURN_SPEED, -TURN_SPEED, TURN_SPEED);
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 1000) {
            telemetry.addData("Turning Left", "");
            telemetry.update();
        }
        robot.setMotorPowers(0, 0,0,0);
        sleep(1000);

        robot.setMotorPowers(FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED, FORWARD_SPEED);
        runtime.reset();

        while (opModeIsActive() && runtime.milliseconds() < 2000) {
            telemetry.addData("Returning Back", "");
            telemetry.update();
        }
        robot.setMotorPowers(0, 0,0,0);
        sleep(1000);
    }

}
