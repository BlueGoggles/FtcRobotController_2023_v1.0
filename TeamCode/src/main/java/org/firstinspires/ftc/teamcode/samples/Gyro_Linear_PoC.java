package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.RobotHardware;

@Autonomous(name = "Gyro PoC", group = "Robot")
public class Gyro_Linear_PoC extends LinearOpMode {

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

        waitForStart();

        turn(90);

        sleep(3000);

        turnTo(-90);
    }

    public void resetAngle() {
        lastAngles = robot.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        currentAngle = 0.0;
    }

    public double getAngle() {
        Orientation currentOrientation = robot.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        double deltaAngle = currentOrientation.firstAngle - lastAngles.firstAngle;

        if (deltaAngle > 180) {
            deltaAngle = deltaAngle - 360;
        } else if (deltaAngle <= -180) {
            deltaAngle = deltaAngle + 360;
        }

        lastAngles = currentOrientation;
        currentAngle = currentAngle + deltaAngle;

        telemetry.addData("Gyro getAngle() - ", deltaAngle);
        telemetry.update();

        return deltaAngle;
    }

    public void turn(double degrees) {
        double error = degrees;

        resetAngle();

        while (opModeIsActive() && Math.abs(error)>2) {
            double motorPower = error < 0 ? -TURN_SPEED : TURN_SPEED;
            robot.setMotorPowers(-motorPower, motorPower, -motorPower, motorPower);

            error = degrees - getAngle();

            telemetry.addData("error", error);
            telemetry.update();
        }

        robot.setMotorPowers(0);
    }

    public void turnTo(double degrees) {
        Orientation currentOrientation = robot.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY,AngleUnit.DEGREES);

        double error = degrees - currentOrientation.firstAngle;

        if (error > 180) {
            error = error - 360;
        } else if (error <= -180) {
            error = error + 360;
        }

        turn(error);
    }

}
