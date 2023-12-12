package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Frontstage Red - Non Corner", group = "FrontstageRedAuton")
public class FrontstageRedNonCorner extends FrontstageRed {

    protected void parkRobot() {

        double inches = 0;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            inches = 10;
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 10 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 10 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        }

        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_FRONT_STAGE_DRIVE_SPEED,  inches);
    }
}