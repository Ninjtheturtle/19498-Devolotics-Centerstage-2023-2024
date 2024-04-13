package org.firstinspires.ftc.teamcode.drive.auton;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class WINGRed extends OpMode {
    private SampleMecanumDrive drive;
    private Servo intakeL, intakeR, lockFront, lockBack, vPitchL, vPitchR, launch, pivot;
    private DcMotorEx liftL, liftR, intake, hang;
    DistanceSensor sensorL, sensorR, senseLF, senseLB;

    // Init Positions
    public final int  INTAKE = 0, VPITCH = 1, VLIFT = 0, LOCKFRONT = 2, LOCKBACK = 3, PIVOT = 5;

    // Veritcal Pitch Servo Postions
    int vPitchIntake = 910;
    int vPitchDeposit = 40;


    // Intake Servo Positions
    int intakeDown = 460;
    int intakeUp = 220;

    // Locking Servo Positions
    int lockFU = 0;
    int lockFD = 250;

    int lockBU = 0;
    int lockBD = 250;

    int pivotHome = 130;
    int pivotScore = 840;

    int pivotPos = pivotHome;

    //Motor Power reduction
    double motorPowerLeft;
    double motorPowerRight;

    //Setting servo variables
    int intakePos = intakeDown;
    int vPitchPos = vPitchIntake;
    int lockFPos = lockFD;
    int lockBPos = lockBD;


    //vTarget positions
    int vMin = 0;
    int vIntake = 40;
    int vTargetInter = 70;
    int targetLow = 500;
    int targetMed = 600;
    int targetHigh = 700;
    int vMax = 1300;

    boolean fIn = false;
    boolean bIn = false;

    //region PIDS
    int vTarget = 0;
    public PIDController vController;
    public double Pv = 0.010877, Iv = 0, Dv = 0.00006, Fv = 0;

    public PIDController vControllerR;
    public double Pvr = 0.010877, Ivr = 0, Dvr = 0.00006, Fvr = 0;

    //AUTONOMOUS PROGRAM:

    // Autonomous Variables
    public List<int[]> PROGRAM = new ArrayList<>();

    int PROPLOCATION_N = 0;
    String PROPLOCATION_S = "left";

    // Autonomous Trajectories
    Pose2d START_POSE = new Pose2d(-37, -64, Math.toRadians(90));
    Trajectory traj_left1 = null;
    Trajectory traj_left201 = null;
    Trajectory traj_left2 = null;
    Trajectory traj_left3 = null;
    Trajectory traj_left4 = null;
    Trajectory traj_left5 = null;
    Trajectory traj_left6 = null;
    Trajectory traj_left7 = null;
    Trajectory traj_left8 = null;

    Trajectory traj_middle1 = null;
    Trajectory traj_middle201 = null;
    Trajectory traj_middle2 = null;
    Trajectory traj_middle3 = null;
    Trajectory traj_middle4 = null;
    Trajectory traj_middle5 = null;
    Trajectory traj_middle6 = null;
    Trajectory traj_middle7 = null;
    Trajectory traj_middle8 = null;

    Trajectory traj_right1 = null;
    Trajectory traj_right201 = null;
    Trajectory traj_right2 = null;
    Trajectory traj_right3 = null;
    Trajectory traj_right4 = null;
    Trajectory traj_right5 = null;
    Trajectory traj_right6 = null;
    Trajectory traj_right7 = null;
    Trajectory traj_right8 = null;

    Trajectory adjust = null;

    // Miscellaneous
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();
    private ElapsedTime timeout = new ElapsedTime();
    public int prevLine = -1;
    public int line = 0;
    public int vTargetTarget = 0;

    // TensorFlow Object Detection
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "bluepropv1.tflite";
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/bluepropv1.tflite";
    private static final String[] LABELS = {
            "BLUE",
    };
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private void Tfod_init() {
        tfod = new TfodProcessor.Builder()
//            .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                .setIsModelTensorFlow2(true)
                .setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }
        builder.addProcessor(tfod);

        visionPortal = builder.build();
        tfod.setMinResultConfidence(0.85f);

        visionPortal.setProcessorEnabled(tfod, true);
    }

    private String Tfod_getLocation() {
        String PROPLOCATION = "left";
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("Objects Detected", currentRecognitions.size());

        if(currentRecognitions.size() == 0) {
            telemetry.addData("Prop Location", PROPLOCATION);
        } else {
            Recognition currentRecognitions_first = currentRecognitions.get(0);
            double x = (currentRecognitions_first.getLeft() + currentRecognitions_first.getRight()) / 2 ;
            double y = (currentRecognitions_first.getTop()  + currentRecognitions_first.getBottom()) / 2 ;

            if(x < 260 && y < 250) {
                PROPLOCATION = "middle";
            } else {
                PROPLOCATION = "right";
            }

            telemetry.addData("Prop Location", PROPLOCATION);
            telemetry.addData("PropX", x);
            telemetry.addData("PropY", y);
        }

        return PROPLOCATION;
    }

    @Override
    public void init() {
        Tfod_init();

        // Declare multiple telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Declare drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //Init Sensors
        senseLF = hardwareMap.get(DistanceSensor.class, "senseLF");
        senseLB = hardwareMap.get(DistanceSensor.class, "senseLB");

        // Init Servos
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

        // Init Motors
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

        intake = hardwareMap.get(DcMotorEx.class, "12");
        intake.setZeroPowerBehavior(BRAKE);

        pivot = hardwareMap.get(Servo.class, "s12");
        pivot.setDirection(Servo.Direction.REVERSE);

        // Init PID controller
        vController = new PIDController(Pv, Iv, Dv);
        vControllerR = new PIDController(Pvr, Ivr, Dvr);

        // Retrieve the IMU from the hardware map
//        IMU imu = hardwareMap.get(IMU.class, "imu");
//        // Adjust the orientation parameters to match your robot
//        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.UP));
//        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
//        imu.initialize(parameters);

        // Set telemetry transmission interval
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void init_loop() {
        // Detect location of team prop
        PROPLOCATION_S = Tfod_getLocation();

        switch (PROPLOCATION_S) {
            case "left":
                PROPLOCATION_N = 0;
                break;
            case "right":
                PROPLOCATION_N = 2;
                break;
            case "middle":
                PROPLOCATION_N = 1;
                break;
        }
    }

    @Override
    public void start() {
        telemetry.addData("Prop Location Number ", PROPLOCATION_S);
        telemetry.update();

        // Close vision portal
        visionPortal.close();

        // AUTONOMOUS TRAJECTORIES

        // Starting position
        drive.setPoseEstimate(START_POSE);

        // Prop on the left
        traj_left1 = drive.trajectoryBuilder(START_POSE)
                .splineTo(new Vector2d(-32, -34), Math.toRadians(320))
                .build(); // spike mark

        traj_left2 = drive.trajectoryBuilder(traj_left1.end())
                .lineToLinearHeading(new Pose2d(-58, -36, Math.toRadians(180)))
                .build(); // align stack

        traj_left201 = drive.trajectoryBuilder(traj_left2.end())
                .forward(0.005)
                .build(); // slowly forward

        traj_left3 = drive.trajectoryBuilder(traj_left201.end(), true)
                .splineToConstantHeading(new Vector2d(-34, -56), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(30, -56))
                .splineToConstantHeading(new Vector2d(46, -33), Math.toRadians(0))
                .build(); // stack

        traj_left4 = drive.trajectoryBuilder(traj_left3.end())
                .splineToConstantHeading(new Vector2d(30, -56), Math.toRadians(180))
                .lineToConstantHeading(new Vector2d(-34, -56))
                .splineToConstantHeading(new Vector2d(-58, -36), Math.toRadians(180))
                .build(); // go backdrop

        traj_left5 = drive.trajectoryBuilder(traj_left4.end(), true)
                .splineToConstantHeading(new Vector2d(-34, -56), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(30, -56))
                .splineToConstantHeading(new Vector2d(46, -33), Math.toRadians(0))
                .build(); // backdrop

        traj_left6 = drive.trajectoryBuilder(traj_left5.end())
                .splineToConstantHeading(new Vector2d(60, -56), Math.toRadians(330))
                .build();

        traj_left7 = drive.trajectoryBuilder(traj_left6.end())
                .lineToLinearHeading(new Pose2d(60, -62, Math.toRadians(180)))
                .build();

        // Prop in the middle
        traj_middle1 = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)))
                .build(); // spike mark

        traj_middle2 = drive.trajectoryBuilder(traj_middle1.end())
                .lineToConstantHeading(new Vector2d(-36, -42))
                .lineToSplineHeading(new Pose2d(-58, -36, Math.toRadians(180)))
                .build(); // move away from spike

        traj_middle201 = drive.trajectoryBuilder(traj_middle2.end())
                .lineToConstantHeading(new Vector2d(-58.01, -36))
                .build(); // move away from spike

        traj_middle3 = drive.trajectoryBuilder(traj_middle201.end())
                .lineToLinearHeading(new Pose2d(-34, -60, Math.toRadians(180)))
                .build(); // stack

        traj_middle4 = drive.trajectoryBuilder(traj_middle3.end())
                .lineToLinearHeading(new Pose2d(30, -60, Math.toRadians(180)))
                .build(); // go backdrop

        traj_middle5 = drive.trajectoryBuilder(traj_middle4.end())
                .lineToLinearHeading(new Pose2d(46, -36, Math.toRadians(180)))
                .build(); // backdrop

        traj_middle6 = drive.trajectoryBuilder(traj_middle5.end())
                .lineToLinearHeading(new Pose2d(40, -60, Math.toRadians(180)))
                .build();

        traj_middle7 = drive.trajectoryBuilder(traj_middle6.end())
                .lineToLinearHeading(new Pose2d(60, -62, Math.toRadians(180)))
                .build();

        // Prop on the right
        traj_right1 = drive.trajectoryBuilder(START_POSE)
                .splineTo(new Vector2d(-30, -36), Math.toRadians(60))
                .build(); // spike mark

        traj_right2 = drive.trajectoryBuilder(traj_right1.end())
                .lineToConstantHeading(new Vector2d(-45, -45))
                .splineToLinearHeading(new Pose2d(-58, -36, Math.toRadians(180)), Math.toRadians(180))
                .build(); // align stack

        traj_right201 = drive.trajectoryBuilder(traj_right2.end())
                .lineToConstantHeading(new Vector2d(-58.01, -36))
                .build(); // align stack

        traj_right3 = drive.trajectoryBuilder(traj_right201.end())
                .lineToLinearHeading(new Pose2d(-34, -60, Math.toRadians(180)))
                .build(); // stack

        traj_right4 = drive.trajectoryBuilder(traj_right3.end())
                .lineToLinearHeading(new Pose2d(30, -60, Math.toRadians(180)))
                .build(); // go backdrop

        traj_right5 = drive.trajectoryBuilder(traj_right4.end())
                .lineToLinearHeading(new Pose2d(46, -39, Math.toRadians(180)))
                .build(); // backdrop

        traj_right6 = drive.trajectoryBuilder(traj_right5.end())
                .lineToLinearHeading(new Pose2d(40, -60, Math.toRadians(180)))
                .build();

        traj_right7 = drive.trajectoryBuilder(traj_right6.end())
                .lineToLinearHeading(new Pose2d(60, -62, Math.toRadians(180)))
                .build();

        // Build Autonomous Program
        buildProgram();
    }

    @Override
    public void loop() {
        //TARGET SETTING
        if (vTarget != vTargetTarget) { vTarget += vTargetTarget; }


        //REGION PID UPDATING
        int vPosition = liftL.getCurrentPosition();
        double vPIDL = vController.calculate(vPosition, vTarget);

        int vPositionR = liftR.getCurrentPosition();
        double vPIDR = vControllerR.calculate(vPositionR, vTarget);

        motorPowerLeft = vPIDL + Fv;
        motorPowerRight = vPIDR + Fvr;

        motorPowerLeft = Math.min(1, Math.max(-0.6, motorPowerLeft));
        motorPowerRight = Math.min(1, Math.max(-0.6, motorPowerRight));

        liftL.setPower(motorPowerLeft);
        liftR.setPower(motorPowerRight);

        // Controller vibrate when both pixels are inside
        if (senseLB.getDistance(DistanceUnit.MM) > 30) {
            bIn = false;
        } else {
            bIn = true;
        }

        if (senseLF.getDistance(DistanceUnit.MM) > 30) {
            fIn = false;
        } else {
            fIn = true;
        }

        boolean detected = bIn && fIn;

        //ENDREGION PID UPDATING
        intakeL.setPosition(((double)intakePos / 1000) - 0.02);
        intakeR.setPosition((double)intakePos / 1000);

        vPitchL.setPosition((double)vPitchPos / 1000);
        vPitchR.setPosition((double)vPitchPos / 1000);

        lockFront.setPosition((double)lockFPos / 1000);
        lockBack.setPosition((double)lockBPos / 1000);

        pivot.setPosition((double)pivotPos / 1000);

        if (line < PROGRAM.size()) {
            int func = PROGRAM.get(line)[0];
            int arg1 = PROGRAM.get(line)[1];
            int arg2 = PROGRAM.get(line)[2];
//            int arg3 = program.get(line)[3];

            boolean CHANGE_LINE = false;

            switch (func) {
                case 1:
                    // setServoPos(int servo, int target1k)
                    switch (arg1) {
                        case INTAKE:
                            intakePos = arg2;
                            break;
                        case VPITCH:
                            vPitchPos = arg2;
                            break;
                        case LOCKFRONT:
                            lockFPos = arg2;
                            break;
                        case LOCKBACK:
                            lockBPos = arg2;
                            break;
                        case PIVOT:
                            pivotPos = arg2;
                            break;
                    }
                    CHANGE_LINE = true;
                    break;
                case 2:
                    // setMotorPos(int motor, int target, int speed)
                    switch (arg1) {
                        case 0:
                            vTargetTarget = arg2;
                            break;
                    }
                    CHANGE_LINE = true;
                    break;
                case 3:
                    /*
                    followTraj(trajNo)

                    Trajectory 1: Spike mark
                    Trajectory 2: Backdrop
                    Trajectory 3: Position to park
                    Trajectory 4: Park
                     */
                    switch (arg1) {
                        case 1:
                            drive.followTrajectoryAsync(traj_left1);
                            break;
                        case 2:
                            drive.followTrajectoryAsync(traj_left2);
                            break;
                        case 3:
                            drive.followTrajectoryAsync(traj_left3);
                            break;
                        case 4:
                            drive.followTrajectoryAsync(traj_left4);
                            break;
                        case 5:
                            drive.followTrajectoryAsync(traj_left5);
                            break;
                        case 6:
                            drive.followTrajectoryAsync(traj_left6);
                            break;
                        case 7:
                            drive.followTrajectoryAsync(traj_left7);
                            break;
                        case 8:
                            drive.followTrajectoryAsync(traj_left8);
                            break;
                        case 11:
                            drive.followTrajectoryAsync(traj_middle1);
                            break;
                        case 12:
                            drive.followTrajectoryAsync(traj_middle2);
                            break;
                        case 13:
                            drive.followTrajectoryAsync(traj_middle3);
                            break;
                        case 14:
                            drive.followTrajectoryAsync(traj_middle4);
                            break;
                        case 15:
                            drive.followTrajectoryAsync(traj_middle5);
                            break;
                        case 16:
                            drive.followTrajectoryAsync(traj_middle6);
                            break;
                        case 17:
                            drive.followTrajectoryAsync(traj_middle7);
                            break;
                        case 18:
                            drive.followTrajectoryAsync(traj_middle8);
                            break;
                        case 21:
                            drive.followTrajectoryAsync(traj_right1);
                            break;
                        case 22:
                            drive.followTrajectoryAsync(traj_right2);
                            break;
                        case 23:
                            drive.followTrajectoryAsync(traj_right3);
                            break;
                        case 24:
                            drive.followTrajectoryAsync(traj_right4);
                            break;
                        case 25:
                            drive.followTrajectoryAsync(traj_right5);
                            break;
                        case 26:
                            drive.followTrajectoryAsync(traj_right6);
                            break;
                        case 27:
                            drive.followTrajectoryAsync(traj_right7);
                            break;
                        case 28:
                            drive.followTrajectoryAsync(traj_right8);
                            break;
                    }
                    CHANGE_LINE = true;
                    break;
                case 5:
                    // waitTime(int ms)
                    if (prevLine != line) {
                        runtime.reset();
                    }
                    if (runtime.milliseconds() > arg1) {
                        CHANGE_LINE = true;
                    }
                    break;
                case 7:
                    // waitTrajDone()
                    if (!drive.isBusy()) {
                        CHANGE_LINE = true;
                    }
                    break;
                case 20:
                    liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    CHANGE_LINE = true;
                    break;
                case 21:
                    switch (arg1) {
                        case 0:
                            vTarget = arg2;
                            break;
                    }
                    CHANGE_LINE = true;
                    break;
                case 69:
                    //Intake Motor
                    intake.setPower((double)-arg1);
                    CHANGE_LINE = true;
                    break;
                case 8:
                    //Check Intaked
                    if (!detected) {
                        if (!fIn) {
                            switch (PROPLOCATION_N) {
                                case 0:
                                    drive.followTrajectoryAsync(traj_left201);
                                    break;
                                case 1:
                                    drive.followTrajectoryAsync(traj_middle201);
                                    break;
                                case 2:
                                    drive.followTrajectoryAsync(traj_right201);
                                    break;
                            }
                            timeout.reset();
                        }
                        else {
                            if (timeout.milliseconds() > 3000) {
                                CHANGE_LINE = true;
                            }
                        }
                    } else {
                        CHANGE_LINE = true;
                    }
                    break;
            }

            prevLine = line;
            if (CHANGE_LINE) {
                line += 1;
            }
        }

        drive.update();

        // TELEMETRY
        telemetry.addData("Line", line);
        telemetry.addData("VPosition", vPosition);
        telemetry.addData("VTarget", vTarget);
        telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS:

    // Hardware Functions
    public void setServoPos(int servo, int target1k) {
        PROGRAM.add(new int[] {1, servo, target1k, 0});
    }

    public void setMotorPos(int motor, int target) {
        PROGRAM.add(new int[] {2, motor, target});
    }
    public void setMotorTarget(int motor, int position) {
        PROGRAM.add(new int[] {21, motor, position, 0});
    }

    public void setServoSpeed(int servo, int target, int speed) {
        PROGRAM.add(new int[] {99, servo, target, speed});
    }

    // Trajectory Functions
    public void followTraj(int trajNo) {
        PROGRAM.add(new int[] {3, trajNo, 0, 0});
    }

    // Wait Functions
    public void waitMotorTarget(int motor, int waitUntilPosition, int sign) {
        PROGRAM.add(new int[] {4, motor, waitUntilPosition, sign});
    }
    public void waitTime(int ms) {
        PROGRAM.add(new int[] {5, ms, 0, 0});
    }
    public void waitMotorTick(int motor, int waitUntilPosition, int sign) {
        PROGRAM.add(new int[] {6, motor, waitUntilPosition, sign});
    }
    public void waitTrajDone() {
        PROGRAM.add(new int[] {7, 0, 0, 0});
    }

    public void setMotorPower(int power) {PROGRAM.add(new int[] {69, power, 0, 0});}

    public void checkIntaked() { PROGRAM.add(new int[] {8, 0, 0, 0}); }

    public void buildProgram() {
        setServoPos(INTAKE, intakeDown);
        setServoPos(LOCKFRONT, lockFD);
        setServoPos(LOCKBACK, lockBD);
        setMotorTarget(VLIFT, 0);
        waitTime(500);

        // Trajectory 1
        switch (PROPLOCATION_N){
            case 0:
                followTraj(1);
                break;
            case 1:
                followTraj(11);
                break;
            case 2:
                followTraj(21);
                break;
        }
        waitTrajDone();

        setServoPos(INTAKE, intakeUp);

        // Trajectory 2
        switch (PROPLOCATION_N){
            case 0:
                followTraj(2);
                break;
            case 1:
                followTraj(12);
                break;
            case 2:
                followTraj(22);
                break;
        }
        waitTime(600);
        setMotorPower(1);
        waitTrajDone();
        waitTime(80);
        checkIntaked();
        setMotorPower(-1);

        // Trajectory 3
        switch (PROPLOCATION_N){
            case 0:
                followTraj(3);
                break;
            case 1:
                followTraj(13);
                break;
            case 2:
                followTraj(23);
                break;
        }
        waitTrajDone();
        setMotorPower(0);

        // Trajectory 4
        switch (PROPLOCATION_N){
            case 0:
                followTraj(4);
                break;
            case 1:
                followTraj(14);
                break;
            case 2:
                followTraj(24);
                break;
        }

        setMotorTarget(VLIFT, targetMed - 70);
        waitTime(500);
        setServoPos(VPITCH, vPitchDeposit);
        waitTime(500);
        setServoPos(PIVOT, pivotScore);
        setMotorTarget(VLIFT, 300);

        // Trajectory 5
        switch (PROPLOCATION_N){
            case 0:
                followTraj(5);
                break;
            case 1:
                followTraj(15);
                break;
            case 2:
                followTraj(25);
                break;
        }
        waitTrajDone();
        setServoPos(LOCKFRONT, lockFU);
        setServoPos(LOCKBACK, lockBU);

        // Trajectory 6
        switch (PROPLOCATION_N){
            case 0:
                followTraj(6);
                break;
            case 1:
                followTraj(16);
                break;
            case 2:
                followTraj(26);
                break;
        }
        waitTime(1000); // adjust for when the robot is far enough away from the backdrop

        setMotorTarget(VLIFT, 600);
        waitTime(200);
        setServoPos(LOCKFRONT, lockFD);
        setServoPos(LOCKBACK, lockBD);
        setServoPos(PIVOT, pivotHome);
        waitTime(400);
        setServoPos(VPITCH, vPitchIntake);
        waitTime(800);
        setMotorTarget(VLIFT, -10);

        // Trajectory 7
        switch (PROPLOCATION_N){
            case 0:
                followTraj(7);
                break;
            case 1:
                followTraj(17);
                break;
            case 2:
                followTraj(27 );
                break;
        }
    }
}