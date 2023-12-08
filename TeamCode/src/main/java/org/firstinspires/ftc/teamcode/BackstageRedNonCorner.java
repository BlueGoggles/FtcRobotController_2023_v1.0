package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Backstage Red - Non Corner", group = "BackstageRedAuton")
public class BackstageRedNonCorner extends BackstageRed {

    protected void parkRobot() {

        double inches = 0;

        if (Utility.getSpikeMark() == Utility.SpikeMark.RIGHT) {
            inches = 14 + (2 * Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES);
        } else if (Utility.getSpikeMark() == Utility.SpikeMark.CENTER) {
            inches = 14 + Constants.DISTANCE_BETWEEN_APRIL_TAG_INCHES;
        } else {
            inches = 14;
        }

        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED,  inches);
    }
}