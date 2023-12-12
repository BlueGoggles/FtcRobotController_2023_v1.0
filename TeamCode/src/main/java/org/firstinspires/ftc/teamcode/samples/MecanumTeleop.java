package org.firstinspires.ftc.teamcode.samples;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MecanumTeleop Testing")
@Disabled
public class MecanumTeleop extends LinearOpMode {

    private IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int FL_Power;
        int FR_Power;
        int BL_Power;
        int BR_Power;
        int Gain_X;
        int Gain_Y;
        int Gain_Z;
        int Max;
        double DB;
        double M;
        double B;
        int Z_;
        double KD;
        double Kp;
        int T_A_;
        double Z__Max;
        float Joystick_X;
        float Joystick_Y;
        float Joystick_Z;
        YawPitchRollAngles Orientation2;
        double ThetaG;
        AngularVelocity Theta_V;
        double Dr;
        double ThetaR;
        double ThetaT;

        imu = hardwareMap.get(IMU.class, "imu");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        FL_Power = 0;
        FR_Power = 0;
        BL_Power = 0;
        BR_Power = 0;
        Gain_X = 1;
        Gain_Y = 1;
        Gain_Z = 1;
        Max = 0;
        DB = 0.05;
        M = 0;
        B = 0;
        Z_ = 0;
        KD = 0.003;
        Kp = 0.024;
        T_A_ = 0;
        Z__Max = 0.75;
        // Initializes the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Creates a Parameters object for use with an IMU in a REV Robotics Control Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD)));
        sleep(100);
        imu.resetYaw();
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                Joystick_X = 1 * gamepad1.right_stick_x;
                Joystick_Y = -1 * gamepad1.right_stick_y;
                Joystick_Z = gamepad1.left_stick_x;
                M = 1 / (1 - DB);
                B = -DB / (1 - DB);
                if (Math.abs(Joystick_X) > DB) {
                    Joystick_X = (float) (M * Joystick_X + B);
                } else {
                    Joystick_X = 0;
                }
                if (Math.abs(Joystick_Y) > DB) {
                    Joystick_Y = (float) (M * Joystick_Y + B);
                } else {
                    Joystick_Y = 0;
                }
                if (Math.abs(Joystick_Z) > DB) {
                    Joystick_Z = (float) (M * Joystick_Z + B);
                } else {
                    Joystick_Z = 0;
                }
                Orientation2 = imu.getRobotYawPitchRollAngles();
                ThetaG = Orientation2.getYaw(AngleUnit.DEGREES);
                Theta_V = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
                Dr = Math.sqrt(Math.pow(Joystick_Y, 2) + Math.pow(Joystick_X, 2));
                ThetaR = Math.atan2(Joystick_Y, Joystick_X) / Math.PI * 180;
                ThetaT = ThetaR - (90 - ThetaG);
                if (gamepad1.dpad_up) {
                    T_A_ = 0;
                }
                if (gamepad1.dpad_right) {
                    T_A_ = -90;
                }
                if (gamepad1.dpad_down) {
                    if (ThetaG < 0) {
                        T_A_ = -180;
                    } else {
                        T_A_ = 180;
                    }
                }
                if (gamepad1.dpad_left) {
                    T_A_ = 90;
                }
                Z_ = (int) ((T_A_ - ThetaG) * Kp - KD * Theta_V.zRotationRate);
                if (Math.abs(Z_) > Z__Max) {
                    Z_ = (int) (Z__Max * (Z_ / Math.abs(Z_)));
                }
//                Joystick_Z = -Z_;
//                Joystick_X = (float) (Math.sin(ThetaT / 180 * Math.PI) * Dr);
//                Joystick_Y = (float) (Math.cos(ThetaT / 180 * Math.PI) * Dr);
                FL_Power = (int) (-Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                FR_Power = (int) (-Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                BL_Power = (int) (Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                BR_Power = (int) (Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
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
                if (Max > 1) {
                    FR_Power = FR_Power / Max;
                    FL_Power = FL_Power / Max;
                    BR_Power = BR_Power / Max;
                    BL_Power = BL_Power / Max;
                }
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                frontLeft.setPower(FL_Power);
                frontRight.setPower(FR_Power);
                // The Y axis of a joystick ranges from -1 in its topmost position
                // to +1 in its bottommost position. We negate this value so that
                // the topmost position corresponds to maximum forward power.
                backLeft.setPower(BL_Power);
                backRight.setPower(BR_Power);
                telemetry.addData("Z Prime", Z_);
                telemetry.addData("Yaw", JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
                telemetry.addData("Velocity", Theta_V.zRotationRate);
                telemetry.addData("Front Left Pow", frontLeft.getPower());
                telemetry.addData("Front Right Pow", frontRight.getPower());
                telemetry.addData("Back Left Pow", backLeft.getPower());
                telemetry.addData("Back Right Pow", backRight.getPower());
                telemetry.update();
            }
        }
    }
}
