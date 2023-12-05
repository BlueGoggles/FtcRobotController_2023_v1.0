package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Backstage Red - Non Corner", group = "BackstageRedAuton")
public class BackstageRedNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.RED;

    @Override
    public void runOpMode() {
        Utility.initializeRobot(robot, color);

        // Drive towards object
        BackstageRed.moveToObject(robot);

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot);

        if (targetFound) {
            Utility.placeSecondPixel(robot);
            Utility.parkRobotNonCorner(robot,color,14);
        } else {
            BackstageRed.targetNotFoundParkRobot(robot);
        }
    }
}