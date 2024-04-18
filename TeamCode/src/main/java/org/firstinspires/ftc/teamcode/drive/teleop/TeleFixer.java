package org.firstinspires.ftc.teamcode.drive.teleop;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
//@Config
public class TeleFixer extends OpMode
{
    // HARDWARE
    private SampleMecanumDrive drive;
    private Servo intakeL, intakeR, lockFront, lockBack, vPitchL, vPitchR, launch, pivot, hangL, hangR;
    private DcMotorEx liftL, liftR, intake, hang;
    private DistanceSensor senseLF, senseLB;

    // Region Other
    private String autoProcess = "none";
    int autoState = 0;
    boolean pHome = false;
    boolean pHigh = false;
    boolean pMed = false;
    boolean pLow = false;

    int resetState = 0;
    boolean pReset = false;
    double resetStartTime = 0;

    // RUNTIME VARIABLES
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();
    private ElapsedTime senseTime = new ElapsedTime();

    // SETTING SERVO POSITIONS
    double intakePos = Values.intakeDown;
    double vPitchPos = Values.vPitchIntake;
    double pivotPos = Values.pivotHome;

    // SLIDE PIDS
    int vTarget = 0;
    public PIDController vController;
    public double Pv = Values.Pv, Iv = Values.Iv, Dv = Values.Dv, Fv = Values.Fv;

    public PIDController vControllerR;
    public double Pvr = Values.Pv, Ivr = Values.Iv, Dvr = Values.Dv, Fvr = Values.Fv;

    // STATES
    double driveSpeed = 1;
    double turnSpeed = 1;
    double motorPowerLeft;
    double motorPowerRight;
    int autoLayer = 300;
    boolean fIn = false;
    boolean bIn = false;
    double prevTime = 0;
    double prevTimeTelemetry = 0;
    boolean prevLockingFront = false,
            prevLockingBack = false,
            prevDetected = false,
            prevAutoUp = false,
            prevAutoDown,
            isLockedFront = false,
            isLockedBack = false,
            isDetected = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Sensors
        senseLF = hardwareMap.get(DistanceSensor.class, "senseLF");
        senseLB = hardwareMap.get(DistanceSensor.class, "senseLB");

        //Servos
        intakeL = hardwareMap.get(Servo.class, "s14");
        intakeL.setDirection(Servo.Direction.REVERSE);
        intakeR = hardwareMap.get(Servo.class, "s1");
        intakeR.setDirection(Servo.Direction.FORWARD);

        lockFront = hardwareMap.get(Servo.class, "s10");
        lockFront.setDirection(Servo.Direction.FORWARD);
        lockBack = hardwareMap.get(Servo.class, "s2");
        lockBack.setDirection(Servo.Direction.REVERSE);

        vPitchL = hardwareMap.get(Servo.class, "s11");
        vPitchL.setDirection(Servo.Direction.FORWARD);
        vPitchR = hardwareMap.get(Servo.class, "s3");
        vPitchR.setDirection(Servo.Direction.REVERSE);

        launch = hardwareMap.get(Servo.class, "s4");

        hangL = hardwareMap.get(Servo.class, "s15");
        hangL.setDirection(Servo.Direction.FORWARD);
        hangR = hardwareMap.get(Servo.class, "s5");
        hangR.setDirection(Servo.Direction.REVERSE);

        pivot = hardwareMap.get(Servo.class, "s12");
        pivot.setDirection(Servo.Direction.REVERSE);

        //Motors
        liftL = hardwareMap.get(DcMotorEx.class, "13");
        liftL.setDirection(DcMotorEx.Direction.FORWARD);
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftL.setZeroPowerBehavior(BRAKE);

        liftR = hardwareMap.get(DcMotorEx.class, "2");
        liftR.setDirection(DcMotorEx.Direction.REVERSE);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setZeroPowerBehavior(BRAKE);

        hang = hardwareMap.get(DcMotorEx.class, "3");
        hang.setDirection(DcMotorEx.Direction.FORWARD);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hang.setZeroPowerBehavior(BRAKE);

        intake = hardwareMap.get(DcMotorEx.class, "12");
        intake.setZeroPowerBehavior(BRAKE);

        // PID controller
        vController = new PIDController(Pv, Iv, Dv);
        vControllerR = new PIDController(Pvr, Ivr, Dvr);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

    }

    @Override
    public void start() {
        totalTime.reset();
    }

    @Override
    public void loop() {
        /////////////// SLIDES PID SECTION \\\\\\\\\\\\\
        int vPosition = liftL.getCurrentPosition();
        double vPID = vController.calculate(vPosition, vTarget);

        int vPositionR = liftR.getCurrentPosition();
        double vPIDR = vControllerR.calculate(vPositionR, vTarget);

//        vController.setPID(Pv, Iv, Dv); // FOR TUNING
//        vControllerR.setPID(Pvr, Ivr, Dvr); // FOR TUNING

        motorPowerLeft = vPID + Fv;
        motorPowerRight = vPIDR + Fvr;
        motorPowerLeft = Math.min(1, Math.max(-0.6, motorPowerLeft));
        motorPowerRight = Math.min(1, Math.max(-0.6, motorPowerRight));
        liftL.setPower(motorPowerLeft);
        liftR.setPower(motorPowerRight);

        /////////////// DRIVE SECTION (FIELD ECCENTRIC) \\\\\\\\\\\\\
        Pose2d poseEstimate = drive.getPoseEstimate();

        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );

        drive.update();

        /////////////// DRIVER CONTROL GP1 \\\\\\\\\\\\\
        // SLIDES MOVEMENT (UP AND DOWN)
        if (gamepad1.right_bumper) {
            vTarget += 15;
        } else if (gamepad1.left_bumper) {
            vTarget -= 15;
        }

        if (gamepad1.a) {
            pivot.setPosition(Values.pivotHome);
        }
        if (gamepad1.b) {
            vPitchL.setPosition(Values.vPitchIntake);
            vPitchR.setPosition(Values.vPitchIntake - 0.02);
        }

        /////////////// TELEMETRY \\\\\\\\\\\\\\\
        double overallTime;
        prevTime = totalTime.milliseconds();
        overallTime = totalTime.milliseconds() - prevTime;

        if (overallTime > 500) {
            telemetry.addData("Deposit Layer", autoLayer);
            telemetry.addData("Intake Position", intakePos);
            telemetry.addData("vPitchPos", vPitchPos);
            telemetry.addData("vPosition", liftL.getCurrentPosition());
            telemetry.addData("vTarget", vTarget);
            telemetry.addData("driveSpeed", driveSpeed);
            telemetry.addData("turnSpeed", turnSpeed);
            telemetry.update();
            prevTimeTelemetry = totalTime.milliseconds();
        }


    }
}