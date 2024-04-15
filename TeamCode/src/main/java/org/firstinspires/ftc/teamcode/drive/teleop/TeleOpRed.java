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
public class TeleOpRed extends OpMode
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

        launch.setPosition(0);
        hangL.setPosition(0);
        hangR.setPosition(0);
    }

    @Override
    public void start() {
        totalTime.reset();
    }

    @Override
    public void loop() {
        /////////////// TIMERS \\\\\\\\\\\\\
        // Endgame Started
        if (totalTime.seconds() > 90 && totalTime.seconds() < 91) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        // Game Ended
        if (totalTime.seconds() > 115 && totalTime.seconds() < 116) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        /////////////// DEPOSIT PIXEL CHECKING \\\\\\\\\\\\\
        bIn = !(senseLB.getDistance(DistanceUnit.MM) > 30);
        fIn = !(senseLF.getDistance(DistanceUnit.MM) > 30);

        boolean detected = bIn && fIn;
        if (detected && !prevDetected) {
            isDetected = !isDetected;
            senseTime.reset();
        }
        prevDetected = detected;
        boolean vibrate = bIn && fIn && senseTime.seconds() < 1;
        if (vibrate) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

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

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                driveSpeed * -gamepad1.left_stick_y,
                driveSpeed * -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading() + Math.toRadians(270));

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        turnSpeed * -gamepad1.right_stick_x
                )
        );

        drive.update();

        /////////////// DRIVER CONTROL GP2 \\\\\\\\\\\\\
        // SPEED THROTTLE
        if (gamepad2.a) {
            turnSpeed = 0.2;
        } else {
            turnSpeed = 1;
        }

        // SLIDES AUTO POSITION CONTROL
        // higher up
        boolean autoUp = gamepad2.dpad_up;
        if (autoUp && !prevAutoUp) {
            autoLayer += 100;
        }
        prevAutoUp = autoUp;

        // lower down
        boolean autoDown = gamepad2.dpad_down;
        if (autoDown && !prevAutoDown) {
            autoLayer -= 100;
        }
        prevAutoDown = autoDown;

        // DRONE CONTROL
        if (gamepad2.y) {
            launch.setPosition(Values.launchReleased);
        }

        // HANG UNLEASH
        if (gamepad2.x) {
            hangL.setPosition(0.4);
            hangR.setPosition(0.4);
        }


        autoLayer = Math.min(Values.vMax, Math.max(300, autoLayer));

        /////////////// DRIVER CONTROL GP1 \\\\\\\\\\\\\
        // SLIDES MOVEMENT (UP AND DOWN)
        if (gamepad1.right_bumper) {
            vTarget += 30;
        } else if (gamepad1.left_bumper) {
            vTarget -= 15;
        }

        // slides positioning
        vTarget = Math.min(Values.vMax, Math.max(-15, vTarget));

        // INTAKE CONTROL
        intake.setPower(1 * ((gamepad1.left_trigger - gamepad1.right_trigger) + Values.intakeKickOut));

        // SETTING SERVO POSITION
        vPitchL.setPosition(vPitchPos);
        vPitchR.setPosition(vPitchPos - 0.02);

        // DEPOSIT LOCK CONTROLS
        // left
        boolean lockingFront = gamepad1.dpad_left;
        if (lockingFront && !prevLockingFront) {
            isLockedFront = !isLockedFront;
        }
        if (isLockedFront) {
            lockFront.setPosition(Values.lockUpFront);
        } else {
            lockFront.setPosition(Values.lockDownFront);
        }
        prevLockingFront = lockingFront;

        // right
        boolean lockingBack = gamepad1.dpad_right;
        if (lockingBack && !prevLockingBack) {
            isLockedBack = !isLockedBack;
        }
        if (isLockedBack) {
            lockBack.setPosition(Values.lockUpBack);
        } else {
            lockBack.setPosition(Values.lockDownBack);
        }
        prevLockingBack = lockingBack;

        // INTAKE POSITION CONTROL
        // manual control
        if (gamepad1.dpad_up) {
            intakePos += 0.05;
        }
        if (gamepad1.dpad_down) {
            intakePos -= 0.05;
        }

        // auto set position
        if (gamepad1.dpad_up && gamepad1.right_trigger != 0) {
            intakePos = Values.intakeUp;
        }
        if (gamepad1.dpad_down && gamepad1.right_trigger != 0) {
            intakePos = Values.intakeDown;
        }

        intakePos = Math.min(Values.intakeDown, Math.max(Values.intakeUp, intakePos));
        intakeL.setPosition(intakePos - 0.02);
        intakeR.setPosition(intakePos);

        // DEPOSIT PIVOT CONTROL
        pivot.setPosition(pivotPos);

        // REGION CONTROLS (FINITE STATE MACHINE)
        boolean low = gamepad1.left_stick_button;
        boolean med = gamepad1.a;
        boolean high = gamepad1.y;
        boolean home = gamepad1.b;

        if (home && !pHome && autoState == 0) {
            autoState = 0;
            autoProcess = "home";
            vTarget = liftL.getCurrentPosition();
        }

        if (low && !pLow && autoState == 0 && vTarget < 10) {
            autoState = 0;
            autoProcess = "low";
            vTarget = liftL.getCurrentPosition();
        }

        switch (autoProcess) {
            case "none":
                if (vTarget < 50) {
                    if (med && !pMed) {
                        autoProcess = "med";
                    }
                }
                break;

            case "low":
                switch (autoState) {
                    case 0:
                        vTarget = -20;
                        runtime.reset();
                        autoState += 1;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 100) {
                            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            vTarget = 0;
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
            case "med":
                switch (autoState) {
                    case 0:
                        vTarget = Values.targetMed;
                        runtime.reset();
                        autoState += 1;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 200) {
                            vPitchPos = Values.vPitchDeposit;
                        }
                        if (runtime.milliseconds() > 200 + 40) {
                            pivotPos = Values.pivotScore;
                        }
                        if (runtime.milliseconds() > 200 + 40 + 100) {
                            vTarget = autoLayer;
                            runtime.reset();
                            autoState += 1;
                        }
                        break;
                    case 2:
                        if (med && !pMed && runtime.milliseconds() > 100) {
                            isLockedFront = true;
                            isLockedBack = true;
//                            vTarget += 40;
                            runtime.reset();
                            autoState += 1;
                        }
                        break;
                    case 3:
                        if (med && !pMed) {
                            pivotPos = Values.pivotHome;
                            isLockedFront = false;
                            isLockedBack = false;
                            runtime.reset();
                            autoState += 1;
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 40 && vTarget >= 600) {
                            vPitchPos = Values.vPitchIntake;
                        }
                        if (runtime.milliseconds() > 0 && vTarget < 600) {
                            vTarget = 600;
                            if (runtime.milliseconds() > 40 + 100) {
                                vPitchPos = Values.vPitchIntake;
                            }
                        }
                        if (runtime.milliseconds() > 60 + 400) {
                            vTarget = -10;
                            runtime.reset();
                            autoState += 1;
                        }
                        break;
                    case 5:
                        if (runtime.milliseconds() > 800) {
                            vTarget = -30;
                        }
                        if (runtime.milliseconds() > 1000) {
                            // reset all encoder positions
                            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            // set encoder to the home position
                            vTarget = 0;
                            // reset the state to none
                            runtime.reset();
                            autoState = 0;
                            autoProcess = "none";
                        }
                        break;
                }
                break;
        }

        pHome = home;
        pLow = low;
        pMed = med;
        pHigh = high;

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