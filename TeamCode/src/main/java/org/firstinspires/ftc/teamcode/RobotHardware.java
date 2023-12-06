package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx intakeWheel = null;
    private DcMotorEx intakeBelt = null;
    private DcMotorEx viperSlide = null;
    private DcMotorEx leadScrew = null;
    private Servo panServo = null;
    private Servo panDoor = null;
    private Servo leadScrewSwitch = null;
    private Servo droneLauncher = null;

    private IMU imu = null;

    private OpenCvCamera controlHubCam = null; // Use OpenCvCamera class from FTC SDK

    private AprilTagProcessor aprilTag = null; // Used for managing the AprilTag detection process.
    private VisionPortal visionPortal = null; // Used to manage the video source.

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_INCH = (Constants.COUNTS_PER_MOTOR_REV * Constants.DRIVE_GEAR_REDUCTION) / (Constants.WHEEL_DIAMETER_INCHES * 3.1415);

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void initialize()    {

        // Drive motors
        leftFront  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_FRONT_LEFT);
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_FRONT_RIGHT);
        leftBack  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_BACK_LEFT);
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_BACK_RIGHT);
        // Expansion hub motors
        intakeWheel  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_INTAKE_WHEEL);
        intakeBelt = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_INTAKE_BELT);
        viperSlide  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_VIPER_SLIDE);
        leadScrew = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_LEAD_SCREW);
        // Servos
        panServo = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_PAN_SERVO);
        panDoor  = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_PAN_DOOR);
        leadScrewSwitch = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_LEAD_SCREW_SWITCH);

        getPanServo().setDirection(Servo.Direction.FORWARD);
        getPanServo().setPosition(Constants.PAN_HOME_POSITION);

        getPanDoor().setDirection(Servo.Direction.FORWARD);
        getPanDoor().setPosition(Constants.MID_SERVO);

        getLeadScrewSwitch().setDirection(Servo.Direction.FORWARD);
        getLeadScrewSwitch().setPosition(0.1);

        getLeftFront().setDirection(DcMotorEx.Direction.REVERSE);
        getRightFront().setDirection(DcMotorEx.Direction.FORWARD);
        getLeftBack().setDirection(DcMotorEx.Direction.REVERSE);
        getRightBack().setDirection(DcMotorEx.Direction.FORWARD);

        getIntakeWheel().setDirection(DcMotorEx.Direction.REVERSE);
        getIntakeBelt().setDirection(DcMotorEx.Direction.FORWARD);
        
        this.getViperSlide().setDirection(DcMotorEx.Direction.REVERSE);
        this.getLeadScrew().setDirection(DcMotorEx.Direction.FORWARD);

        this.getViperSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.getLeadScrew().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.getViperSlide().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.getLeadScrew().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        setMotorPowers(Constants.ZERO_POWER);
    }

    public void initializeDroneLauncher()    {

        droneLauncher = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_DRONE_LAUNCHER);

        getDroneLauncher().setDirection(Servo.Direction.FORWARD);
        getDroneLauncher().setPosition(0.6);
    }

    public void setMotorPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        this.getLeftFront().setPower(leftFront);
        this.getRightFront().setPower(rightFront);
        this.getLeftBack().setPower(leftBack);
        this.getRightBack().setPower(rightBack);
    }

    public void setMotorPowers(double allWhealPower) {
        setMotorPowers(allWhealPower, allWhealPower, allWhealPower, allWhealPower);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        getLeftFront().setMode(mode);
        getRightFront().setMode(mode);
        getLeftBack().setMode(mode);
        getRightBack().setMode(mode);
    }

    public void setZeroPowerBehavior() {
        getLeftFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getLeftBack().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightBack().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPosition(Utility.Direction direction, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {

        int leftFrontTarget = getLeftFront().getCurrentPosition();
        int rightFrontTarget= getRightFront().getCurrentPosition();
        int leftBackTarget = getLeftBack().getCurrentPosition();
        int rightBackTarget = getRightBack().getCurrentPosition();

        // Determine new target position, and pass to motor controller
        if (direction == Utility.Direction.FORWARD) {
            leftFrontTarget = leftFrontTarget + (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget + (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget + (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget + (int) (rightBackInches * COUNTS_PER_INCH);
        } else if (direction == Utility.Direction.BACKWARD) {
            leftFrontTarget = leftFrontTarget - (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget - (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget - (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget - (int) (rightBackInches * COUNTS_PER_INCH);
        } else if (direction == Utility.Direction.LEFT) {
            leftFrontTarget = leftFrontTarget - (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget + (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget + (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget - (int) (rightBackInches * COUNTS_PER_INCH);
        } else if (direction == Utility.Direction.RIGHT) {
            leftFrontTarget = leftFrontTarget + (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget - (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget - (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget + (int) (rightBackInches * COUNTS_PER_INCH);
        }

        // Set the Target Position
        getLeftFront().setTargetPosition(leftFrontTarget);
        getRightFront().setTargetPosition(rightFrontTarget);
        getLeftBack().setTargetPosition(leftBackTarget);
        getRightBack().setTargetPosition(rightBackTarget);
    }

    public void setTargetPosition(Utility.Direction direction, double inches) {

        setTargetPosition(direction, inches, inches, inches, inches);
    }

    public void initializeIMU() {

        setZeroPowerBehavior();

        // Control Hub IMU Parameters
//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        // Expansion Hub IMU Parameters
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu = myOpMode.hardwareMap.get(IMU.class, Constants.EXPANSION_IMU);
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double getCurrentHeading() {
        YawPitchRollAngles orientation = getImu().getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getAbsoluteAngle() {
        return getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    public void initializeOpenCV(OpenCvPipeline pipeline) {

        // Create an instance of the camera
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        this.controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                myOpMode.hardwareMap.get(WebcamName.class, Constants.DEVICE_CAMERA), cameraMonitorViewId);

        this.controlHubCam.setPipeline(pipeline);

        this.controlHubCam.openCameraDevice();
        this.controlHubCam.startStreaming(Constants.CAMERA_WIDTH, Constants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    public void releaseResourcesForOpenCV() {

        getControlHubCam().stopRecordingPipeline();
        getControlHubCam().stopStreaming();
        getControlHubCam().closeCameraDevice();
    }

    public void initializeAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, Constants.DEVICE_CAMERA))
                .addProcessor(aprilTag)
                .build();
    }

    public int getLeadScrewPosition() {
        return this.getLeadScrew().getCurrentPosition();
    }

    public int getViperSlidePosition() {
        return this.getViperSlide().getCurrentPosition();
    }

    public LinearOpMode getMyOpMode() {
        return this.myOpMode;
    }

    public DcMotorEx getLeftFront() {
        return this.leftFront;
    }

    public DcMotorEx getRightFront() {
        return this.rightFront;
    }

    public DcMotorEx getLeftBack() {
        return this.leftBack;
    }

    public DcMotorEx getRightBack() {
        return this.rightBack;
    }

    public DcMotorEx getIntakeWheel() {
        return this.intakeWheel;
    }

    public DcMotorEx getIntakeBelt() {
        return this.intakeBelt;
    }

    public DcMotorEx getViperSlide() {
        return this.viperSlide;
    }

    public DcMotorEx getLeadScrew() {
        return this.leadScrew;
    }

    public Servo getPanServo() {
        return this.panServo;
    }

    public Servo getLeadScrewSwitch() {
        return leadScrewSwitch;
    }

    public Servo getPanDoor() {
        return this.panDoor;
    }

    public Servo getDroneLauncher() {
        return this.droneLauncher;
    }

    public IMU getImu() {
        return this.imu;
    }

    public OpenCvCamera getControlHubCam() {
        return this.controlHubCam;
    }

    public AprilTagProcessor getAprilTag() {
        return this.aprilTag;
    }

    public VisionPortal getVisionPortal() {
        return this.visionPortal;
    }
}
