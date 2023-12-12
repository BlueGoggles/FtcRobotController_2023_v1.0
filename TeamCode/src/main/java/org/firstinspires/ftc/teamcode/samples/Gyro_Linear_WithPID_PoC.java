package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Gyro Linear WithPID PoC", group = "Samples")
@Disabled
public class Gyro_Linear_WithPID_PoC extends LinearOpMode {

    private RobotHardware robot = new RobotHardware(this);

    private ElapsedTime runtime = new ElapsedTime();

    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0.0;

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        robot.initialize();
        robot.initializeIMU();

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        turnPID(90);

        sleep(3000);

        turnToPID(-90);
    }

    public double getAbsoluteAngle() {
        return robot.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY,AngleUnit.DEGREES).firstAngle;
    }
    public void turnPID(double degrees) {
        turnToPID(degrees + getAbsoluteAngle());
    }

    void turnToPID(double targetAngle) {
        PIDController pid = new PIDController(targetAngle, 0.01, 0, 0.003);
        telemetry.setMsTransmissionInterval(50);

        // Checking lastSlope to make sure that it's not oscillating when it quits
        while (opModeIsActive() && (Math.abs(targetAngle - getAbsoluteAngle()) > 1 || pid.getLastSlope() > 0.75)) {
            double motorPower = pid.update(getAbsoluteAngle());
            robot.setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);

            telemetry.addData("Current Angle", getAbsoluteAngle());
            telemetry.addData("Target Angle", targetAngle);
            telemetry.addData("Slope", pid.getLastSlope());
            telemetry.addData("Power", motorPower);
            telemetry.update();
        }
        robot.setMotorPowers(0);
    }
}
