package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Backstage Blue - Non Corner", group = "BackstageBlueAuton")
public class BackstageBlueNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        BackstageBlue.blueAuton(robot, false);
    }
}