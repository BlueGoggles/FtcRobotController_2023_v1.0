package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    }
}
