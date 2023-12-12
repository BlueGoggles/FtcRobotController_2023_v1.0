package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Backstage Blue - Non Corner", group = "BackstageBlueAuton")
public class BackstageBlueNonCorner extends BackstageBlue {

    protected void parkRobot() {

        double inches = 0;

        if (Utility.getSpikeMark() == Utility.SpikeMark.LEFT) {
            inches = 8 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 8 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 8;
        }

        Utility.encoderDrive(robot, Utility.Direction.RIGHT, Constants.AUTON_DRIVE_SPEED, inches * Constants.STRAFE_MOVEMENT_RATIO);
        Utility.turnToPID(robot, 0);
    }
}