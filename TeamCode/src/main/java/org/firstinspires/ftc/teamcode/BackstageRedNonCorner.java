package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Backstage Red - Non Corner", group = "BackstageRedAuton")
public class BackstageRedNonCorner extends BackstageRed {

    protected void parkRobot() {

        double inches = 0;

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            inches = 8 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 8 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 8;
        }

        Utility.encoderDrive(robot, Utility.Direction.LEFT, Constants.AUTON_DRIVE_SPEED, inches * Constants.STRAFE_MOVEMENT_RATIO);
        Utility.turnToPID(robot, 0);
    }
}