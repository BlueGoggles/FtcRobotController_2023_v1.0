package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double FL_Power;
        double FR_Power;
        double BL_Power;
        double BR_Power;
        double Gain_X;
        double Gain_Y;
        double Gain_Z;
        double Max;
        double Deadband;
        double M;
        double B;
        double Z_;
        double KD;
        double Kp;
        int Target_Angle;
        double Z__Max;
        double Joystick_X;
        double Joystick_Y;
        double Joystick_Z;
        YawPitchRollAngles Orientation2;
        double Theta_Actual;
        AngularVelocity Theta_Velocity;
        double Speed;
        double Theta_Request;
        double Theta_Command;
        double Error2;
        boolean enableManualOverride;
        double teleOpSpeed;

        // By default we don't want to allow the lead screw to run until it has been released.
        boolean allowLeadScrew = false;
        // By default we don't want to allow the drone to be launched until the timer has elapsed.
        boolean allowDroneLauncher = false;

        // Create the timer to lock out the drone launcher.
        //ElapsedTime droneLauncherWaitTimer = new ElapsedTime();

        RobotHardware robot = new RobotHardware(this);
        robot.initialize();
        robot.initializeIMU();
        robot.initializeDroneLauncher();
        sleep(100);

        FL_Power = 0;
        FR_Power = 0;
        BL_Power = 0;
        BR_Power = 0;
        Gain_X = 1;
        Gain_Y = 1;
        Gain_Z = 1;
        Max = 0;
        Deadband = 0.05;
        M = 0;
        B = 0;
        Z_ = 0;
        KD = 0.003;
        Kp = 0.024;
        Target_Angle = 0;
        Z__Max = 0.75;
        enableManualOverride = false;
        teleOpSpeed = 0.0;
        robot.getImu().resetYaw();

        waitForStart();

        if (opModeIsActive()) {
            // Reset the timer to start after "Start" is pressed.
            //droneLauncherWaitTimer.reset();

            while (opModeIsActive()) {
                Joystick_X = -1 * gamepad1.right_stick_x;
                Joystick_Y = -1 * gamepad1.right_stick_y;
                Joystick_Z = gamepad1.left_stick_x;
                M = 1 / (1 - Deadband);
                B = -Deadband / (1 - Deadband);
                if (Math.abs(Joystick_X) > Deadband) {
                    Joystick_X = (float) (M * Joystick_X + B);
                } else {
                    Joystick_X = 0;
                }
                if (Math.abs(Joystick_Y) > Deadband) {
                    Joystick_Y = (float) (M * Joystick_Y + B);
                } else {
                    Joystick_Y = 0;
                }
                if (Math.abs(Joystick_Z) > Deadband) {
                    Joystick_Z = (float) (M * Joystick_Z + B);
                } else {
                    Joystick_Z = 0;
                }
                Orientation2 = robot.getImu().getRobotYawPitchRollAngles();
                Theta_Actual = Double.parseDouble(JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
                Theta_Velocity = robot.getImu().getRobotAngularVelocity(AngleUnit.DEGREES);
                Speed = Math.sqrt(Math.pow(Joystick_Y, 2) + Math.pow(Joystick_X, 2));
                Theta_Request = Math.atan2(Joystick_Y, Joystick_X) / Math.PI * 180;
                Theta_Command = Theta_Request - (90 - Theta_Actual);
                if (gamepad1.dpad_up) {
                    Target_Angle = 0;
                }
                if (gamepad1.dpad_right) {
                    Target_Angle = -90;
                }
                if (gamepad1.dpad_left) {
                    Target_Angle = 90;
                }
                if (gamepad1.dpad_down) {
                    if (Theta_Actual < 0) {
                        Target_Angle = -180;
                    } else {
                        Target_Angle = 180;
                    }
                }
                if (Math.abs(Target_Angle - Theta_Actual) < 180) {
                    Error2 = (int) (Target_Angle - Theta_Actual);
                } else {
                    if (Target_Angle - Theta_Actual < 0) {
                        Error2 = (int) (Target_Angle - (Theta_Actual - 360));
                    } else {
                        Error2 = (int) (Target_Angle - (Theta_Actual + 360));
                    }
                }
                Z_ = (Error2 * Kp - KD * Theta_Velocity.zRotationRate);
                if (Math.abs(Z_) > Z__Max) {
                    Z_ = (Z__Max * (Z_ / Math.abs(Z_)));
                }

                if( enableManualOverride ) {
                    // Leave Joystick_Z alone.
                } else {
                    Joystick_Z = -Z_;
                }

                Joystick_X = (Math.sin(Theta_Command / 180 * Math.PI) * Speed);
                Joystick_Y = (Math.cos(Theta_Command / 180 * Math.PI) * Speed);
                FL_Power = (-Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                FR_Power = (-Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                BL_Power = (Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                BR_Power = (Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                if (Math.abs(FR_Power) > Math.abs(FL_Power)) {
                    Max = Math.abs(FR_Power);
                } else {
                    Max = Math.abs(FL_Power);
                }
                if (Math.abs(BL_Power) > Max) {
                    Max = Math.abs(BL_Power);
                }
                if (Math.abs(BR_Power) > Max) {
                    Max = Math.abs(BR_Power);
                }

                if (gamepad1.left_trigger > Constants.ZERO_POWER) {
                    teleOpSpeed = Constants.TELEOP_MODIFIED_SPEED;
                } else {
                    teleOpSpeed = Constants.TELEOP_DEFAULT_SPEED;
                }

                if (Max > teleOpSpeed) {
                    FR_Power = (FR_Power * teleOpSpeed) / Max;
                    FL_Power = (FL_Power * teleOpSpeed) / Max;
                    BR_Power = (BR_Power * teleOpSpeed) / Max;
                    BL_Power = (BL_Power * teleOpSpeed) / Max;
                }

                robot.setMotorPowers(-FL_Power, FR_Power, -BL_Power, BR_Power);

                int intakePower = 0;
                if( gamepad2.x ) {
                    intakePower = 1;
                } else if(gamepad2.back) {
                    intakePower = -1;
                }

                robot.getIntakeWheel().setPower(intakePower);
                robot.getIntakeBelt().setPower(intakePower);
                robot.getIntakeWheel().setPower(intakePower);
                robot.getIntakeBelt().setPower(intakePower);

                // NOTE: This program is single threaded right now. So we can't do multiple operations at once.
                // Use this function to check and see if the viper slide or lead screw need to be stopped.
                Utility.checkSlideAndScrewMotors(robot);
                // Use this function to ensure that pan is moved to the correct position without a blocking while loop.
                Utility.checkPanPosition(robot);
                // We're not using this right now.
                /*
                if( droneLauncherWaitTimer.time() > Constants.END_GAME_LOCKOUT_TIME ) {
                    allowDroneLauncher = true;
                }
                */

                // This variable controls whether we are manually steering or auto steering.
                if( gamepad1.back ) {
                    enableManualOverride = !enableManualOverride;
                }

                // Check to see if we can allow the lead screw to go. It needs to be released before we can control it.
                if( allowLeadScrew ) {
                    // Extend the lead screw.
                    if (gamepad1.left_bumper) {
                        Utility.controlLeadScrew(robot, DcMotorEx.Direction.FORWARD);
                    }

                    // Retract the lead screw.
                    if (gamepad1.right_bumper) {
                        Utility.controlLeadScrew(robot, DcMotorEx.Direction.REVERSE);
                    }
                }

                // Control the viper slide.
                if (gamepad2.a) {
                    Utility.panHome(robot);
                    Utility.resetViperSlide(robot);
                }

                if (gamepad2.b) {
                    Utility.extendViperSlide(robot,false, Utility.StageLocations.BACK);
                    Utility.panDelivery(robot);
                }

                if (gamepad2.y) {
                    Utility.retractViperSlide(robot);
                }

                if( gamepad2.left_trigger > 0.5 ) {
                    Utility.nudgeViperSlide(robot, Utility.ViperSlideDirection.DOWN);
                }

                if( gamepad2.right_trigger > 0.5 ) {
                    Utility.nudgeViperSlide(robot, Utility.ViperSlideDirection.UP);
                }

                if ( gamepad1.a ) {
                    robot.getLeadScrewSwitch().setPosition(0.4);
                    // Tell the program that it's okay to control the lead screw.
                    allowLeadScrew = true;
                }

                if (gamepad2.left_bumper) {
                    robot.getPanDoor().setPosition(Constants.PAN_DOOR_START_POSITION);
                } else {
                    robot.getPanDoor().setPosition(Constants.PAN_DOOR_STOP_POSITION);
                }

                if ( gamepad1.start ) {
                    robot.getDroneLauncher().setPosition(0.4);
                }

                // Press this button to reset the yaw during Teleop. Only allow this to happen if we are in manual mode.
                if (gamepad1.y && enableManualOverride) {
                    robot.getImu().resetYaw();
                }

                telemetry.addData("Z Prime", Z_);
                telemetry.addData("Yaw", JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
                telemetry.addData("Velocity", Theta_Velocity.zRotationRate);
                telemetry.addData("Front Left Pow", robot.getLeftFront().getPower());
                telemetry.addData("Front Right Pow", robot.getRightFront().getPower());
                telemetry.addData("Back Left Pow", robot.getLeftBack().getPower());
                telemetry.addData("Back Right Pow", robot.getRightBack().getPower());

                telemetry.addData("Intake Wheel Power (gamepad2.rightTrigger[forward]/leftTrigger[backword])", robot.getIntakeWheel().getPower());
                telemetry.addData("Intake Belt Power (gamepad2.rightTrigger[forward]/leftTrigger[backword])", robot.getIntakeBelt().getPower());

                telemetry.addData("Lead Screw Position", robot.getLeadScrewPosition());
                telemetry.addData("Viper Slide Encoder Position", robot.getViperSlidePosition());
                telemetry.addData("Viper Current Position", Utility.getViperSlideCurrentState());
                telemetry.addData("Viper Requested Position", Utility.getViperSlideRequestedState());
                telemetry.addData("Lead Screw Forward (gamepad1.left_bumper)", gamepad1.left_bumper);
                telemetry.addData("Lead Screw Reverse (gamepad1.right_bumper)", gamepad1.right_bumper);

                telemetry.addData("Pan Home & Reset Viper Slide (gamepad2.a)", gamepad2.a);
                telemetry.addData("Extend Viper Slide & Pan Delivery (gamepad2.b)", gamepad2.b);
                telemetry.addData("Retract Viper Slide & Pan Delivery (gamepad2.y)", gamepad2.y);

                telemetry.addData("Lead Screw Switch release (gamepad1.a)", gamepad1.a);

                telemetry.addData("Pan Door start/stop (gamepad2.left_bumper)", gamepad2.left_bumper);

                telemetry.addData("Drone Launcher Position", robot.getDroneLauncher().getPosition());

                telemetry.addData("Joystick Z (gamepage1.left_stick_x", Joystick_Z);

                telemetry.update();
            }
        }
    }
}