package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Frontstage Red - Non Corner", group = "FrontstageRedAuton")
public class FrontstageRedNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.RED;

    @Override
    public void runOpMode() {
        Utility.initializeRobot(robot, color);

        // Drive towards object
        FrontstageRed.moveToObject(robot);

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot);

        if (targetFound) {
            Utility.placeSecondPixel(robot);
            Utility.parkRobotNonCorner(robot,color,14);
        } else {
            FrontstageRed.targetNotFoundParkRobot(robot);
        }
    }
}