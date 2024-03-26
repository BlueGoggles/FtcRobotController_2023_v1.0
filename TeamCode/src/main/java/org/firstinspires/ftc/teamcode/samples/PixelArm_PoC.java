package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotHardware;
import org.firstinspires.ftc.teamcode.Utility;

@Autonomous(name = "PixelArm_PoC")
public class PixelArm_PoC extends LinearOpMode  {

    //protected RobotHardware robot = new RobotHardware(this);
    public void pixelArmProgram(RobotHardware robot) {
/*
        // Initialize Robot with Encoder
        robot.initialize();
        robot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Initialize Gyro sensor
        robot.initializeIMU();

        waitForStart();
*/
       // if(opModeIsActive()) {

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_PIXEL_ARM_MAIN_SPEED, 8.75);

            Utility.deployPixelArm(robot);

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_PIXEL_ARM_BACKUP_SPEED, 4.5);


            Utility.resetPixel_Arm(robot);

            robot.getIntakeWheel().setPower(1);
            robot.getIntakeBelt().setPower(1);

            Utility.encoderDrive(robot, Utility.Direction.BACKWARD, Constants.AUTON_PIXEL_ARM_PICKUP_SPEED, 4.5);

            robot.getMyOpMode().sleep(750);

            Utility.encoderDrive(robot, Utility.Direction.FORWARD, Constants.AUTON_PIXEL_ARM_MAIN_SPEED, 8.75);

            robot.getIntakeWheel().setPower(0);
            robot.getIntakeBelt().setPower(0);

        //}
    }

    @Override
    public void runOpMode() throws InterruptedException {

    }
}