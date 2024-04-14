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
    //HARDWARE
    private SampleMecanumDrive drive;
    private Servo intakeL, intakeR, lockFront, lockBack, vPitchL, vPitchR, launch, pivot, hangL, hangR;
    private DcMotorEx liftL, liftR, intake, hang;
    DistanceSensor sensorL, sensorR, senseLF, senseLB;

    //Region Other
    private String autoProcess = "none";
    int autoState = 0;
    boolean pHome = false;
    boolean pHigh = false;
    boolean pMed = false;
    boolean pLow = false;
    boolean pDetected = false;

    int resetState = 0;
    boolean pReset = false;
    double resetStartTime = 0;

    //Runtime Vars
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();
    private ElapsedTime senseTime = new ElapsedTime();

    //Drive Speed
    double driveSpeed = 1;
    double turnSpeed = 1;

    //Backdrop Sensors
    boolean nearBack, nearRight, nearLeft = false;
    int state = 0;
    int backDistanceThreshold = 25;

    //vPitch servo positions
    double vPitchIntake = 0.96;
    double vPitchDeposit = 0.04;

    //intake servo positions
    double intakeDown = 0.45;
    double intakeUp = 0.12;

    //locking servo positions
    double lockUpFront = 0;
    double lockDownFront = 0.25;

    double lockDownBack = 0.25;
    double lockUpBack = 0;

    //pivot servo positions
    double pivotHome = 0.13;
    double pivotScore = 0.84;

    //Setting servo variables
    double intakePos = intakeDown;
    double vPitchPos = vPitchIntake;
    double pivotPos = pivotHome;
    double lockAngleFront = 0;
    double lockAngleBack = 0;

    double intakeKickOut = 0;
    double kickoutSpeed = -1;

    double hangFeed = 0.4;
    boolean isHang = false;

    int turing = 1;

    //region PIDS
    int vTarget = 0;
    public PIDController vController;
    public double Pv = 0.010877, Iv = 0, Dv = 0.00006, Fv = 0;

    public PIDController vControllerR;
    public double Pvr = 0.010877, Ivr = 0, Dvr = 0.00006, Fvr = 0;

    int vMin = 0;
    int vIntake = 40;
    int vTargetInter = 70;
    int targetLow = 500;
    int targetMed = 600;
    int targetHigh = 700;
    int vMax = 1300;

    double launched = 0.2;

    double motorPowerLeft;
    double motorPowerRight;

    int autoLayer = 300;
    boolean fIn = false;
    boolean bIn = false;

    double prevTime = 0;
    double prevTimeTelemetry = 0;
    //Locking
    boolean prevLockingFront = false, prevLockingBack = false, prevIntakePitch = false, prevHanging = false, prevDetected = false, prevAutoUp = false, prevAutoDown;
    boolean isLockedFront = false, isLockedBack = false, isIntakePitch = true, isHanging = true, isDetected = true, isAutoUp = true, isAutoDown = true;


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

        hangL = hardwareMap.get(Servo.class, "s15");
        hangL.setDirection(Servo.Direction.FORWARD);
        hangR = hardwareMap.get(Servo.class, "s5");
        hangR.setDirection(Servo.Direction.REVERSE);

        lockFront = hardwareMap.get(Servo.class, "s10");
        lockFront.setDirection(Servo.Direction.FORWARD);
        lockBack = hardwareMap.get(Servo.class, "s2");
        lockBack.setDirection(Servo.Direction.REVERSE);

        vPitchL = hardwareMap.get(Servo.class, "s11");
        vPitchL.setDirection(Servo.Direction.FORWARD);
        vPitchR = hardwareMap.get(Servo.class, "s3");
        vPitchR.setDirection(Servo.Direction.REVERSE);

        launch = hardwareMap.get(Servo.class, "s4");

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
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        launch.setPosition(0);
        hangL.setPosition(0);
        hangR.setPosition(0);
    }

    public void start() {
        totalTime.reset();
    }

    @Override
    public void loop() {
        // Endgame started
        if (totalTime.seconds() > 90 && totalTime.seconds() < 91) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        // Game ended
        if (totalTime.seconds() > 115 && totalTime.seconds() < 116) {
            gamepad1.rumble(200);
            gamepad2.rumble(200);
        }

        // Controller vibrate when both pixels are inside
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

        // PID Stuff
//        vController.setPID(Pv, Iv, Dv); //comment out after done tuning
        int vPosition = liftL.getCurrentPosition();
        double vPID = vController.calculate(vPosition, vTarget);
//        vControllerR.setPID(Pvr, Ivr, Dvr); //comment out after done tuning
        int vPositionR = liftR.getCurrentPosition();
        double vPIDR = vControllerR.calculate(vPositionR, vTarget);
        motorPowerLeft = vPID + Fv;
        motorPowerRight = vPIDR + Fvr;
        motorPowerLeft = Math.min(1, Math.max(-0.6, motorPowerLeft));
        motorPowerRight = Math.min(1, Math.max(-0.6, motorPowerRight));
        liftL.setPower(motorPowerLeft);
        liftR.setPower(motorPowerRight);

//        Pose2d poseEstimate = new Pose2d(Math.toRadians(90));
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

        // Update everything. Odometry. Etc.
        drive.update();

//        Throttle turning speed with gamepad 2
        if (gamepad2.a) {
            turnSpeed = 0.4;
        } else {
            turnSpeed = 1;
        }

        //vTarget Up Down
        if (gamepad1.right_bumper) {
            vTarget += 30;
        } else if (gamepad1.left_bumper) {
            vTarget -= 15;
        }
        //vTarget Max Min
        vTarget = Math.min(vMax, Math.max(-15, vTarget));

        //setting Servo Positions
        vPitchL.setPosition(vPitchPos);
        vPitchR.setPosition(vPitchPos - 0.02);

        //Intake
        intake.setPower(1 * ((gamepad1.left_trigger - gamepad1.right_trigger) + intakeKickOut));

        //Auto Up
        boolean autoUp = gamepad2.dpad_up;
        if (autoUp && !prevAutoUp) {
            autoLayer += 100;
        }
        prevAutoUp = autoUp;

        //Auto Down
        boolean autoDown = gamepad2.dpad_down;
        if (autoDown && !prevAutoDown) {
            autoLayer -= 100;
        }
        prevAutoDown = autoDown;

        autoLayer = Math.min(vMax, Math.max(300, autoLayer));

        //Locking Left
        boolean lockingFront = gamepad1.dpad_left;
        if (lockingFront && !prevLockingFront) {
            isLockedFront = !isLockedFront;
        }
        if (isLockedFront) {
            lockFront.setPosition(lockUpFront);
        } else {
            lockFront.setPosition(lockDownFront);
        }
        prevLockingFront = lockingFront;

        //Locking Right
        boolean lockingBack = gamepad1.dpad_right;
        if (lockingBack && !prevLockingBack) {
            isLockedBack = !isLockedBack;
        }
        if (isLockedBack) {
            lockBack.setPosition(lockUpBack);
        } else {
            lockBack.setPosition(lockDownBack);
        }
        prevLockingBack = lockingBack;

        if (gamepad1.dpad_up) {
            intakePos += 0.05;
        }
        if (gamepad1.dpad_down) {
            intakePos -= 0.05;
        }

        if (gamepad1.dpad_up && gamepad1.right_trigger != 0) {
            intakePos = intakeUp;
        }
        if (gamepad1.dpad_down && gamepad1.right_trigger != 0) {
            intakePos = intakeDown;
        }

        intakePos = Math.min(intakeDown, Math.max(intakeUp, intakePos));
        intakeL.setPosition(intakePos - 0.02);
        intakeR.setPosition(intakePos);

        //Pivot Servo Control
        pivot.setPosition(pivotPos);

        //Drone Launch
        if (gamepad2.y) {
            launch.setPosition(launched);
        }

        if (gamepad2.x) {
            hangL.setPosition(0.4);
            hangR.setPosition(0.4);
        }

        //region controls
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
                        vTarget = targetMed;
                        runtime.reset();
                        autoState += 1;
                        break;
                    case 1:
                        if (runtime.milliseconds() > 200) {
                            vPitchPos = vPitchDeposit;
                        }
                        if (runtime.milliseconds() > 200 + 40) {
                            pivotPos = pivotScore;
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
                            pivotPos = pivotHome;
                            isLockedFront = false;
                            isLockedBack = false;
                            runtime.reset();
                            autoState += 1;
                        }
                        break;
                    case 4:
                        if (runtime.milliseconds() > 40 && vTarget >= 600) {
                            vPitchPos = vPitchIntake;
                        }
                        if (runtime.milliseconds() > 0 && vTarget < 600) {
                            vTarget = 600;
                            if (runtime.milliseconds() > 40 + 100) {
                                vPitchPos = vPitchIntake;
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
                            //reset all encoder positions
                            liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                            liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            //set encoder to the home position
                            vTarget = 0;
                            //reset the state to none
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

        //Display Telemetry
        double overallTime;
        overallTime = totalTime.milliseconds() - prevTime;
        prevTime = totalTime.milliseconds();

        if (totalTime.milliseconds() - prevTimeTelemetry > 500) {
            telemetry.addData("Deposit Layer", autoLayer);
            telemetry.addData("Intake Position", intakePos);
            telemetry.addData("vPitchPos", vPitchPos);
            telemetry.addData("vPosition", liftL.getCurrentPosition());
            telemetry.addData("vTarget", vTarget);
            telemetry.addData("state", state);
            telemetry.addData("driveSpeed", driveSpeed);
            telemetry.addData("turnSpeed", turnSpeed);
            telemetry.update();
            prevTimeTelemetry = totalTime.milliseconds();
        }


    }
}