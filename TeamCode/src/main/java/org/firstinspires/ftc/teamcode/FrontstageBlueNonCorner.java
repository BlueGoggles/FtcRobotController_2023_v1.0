package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Frontstage Blue - Non Corner", group = "FrontstageBlueAuton")
public class FrontstageBlueNonCorner extends LinearOpMode {

    RobotHardware robot = new RobotHardware(this);
    private Utility.Color color = Utility.Color.BLUE;

    @Override
    public void runOpMode() {
        Utility.initializeRobot(robot, color);

        // Drive towards object
        FrontstageBlue.moveToObject(robot);

        // Move to desired AprilTag
        boolean targetFound = Utility.moveToAprilTag(robot);

        if (targetFound) {
            Utility.placeSecondPixel(robot);
            Utility.parkRobotNonCorner(robot,color,26);
        } else {
            FrontstageBlue.targetNotFoundParkRobot(robot);
        }
    }
}