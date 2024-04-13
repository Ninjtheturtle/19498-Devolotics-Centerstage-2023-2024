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
public class Auton_Template extends OpMode {
    // Declaring Hardware
    private SampleMecanumDrive drive;

    // Autonomous Variables
    public List<int[]> PROGRAM = new ArrayList<>();

    // Autonomous Trajectories
    Pose2d START_POSE = new Pose2d(12, 63, Math.toRadians(270));
    Trajectory traj_left1 = null;
    Trajectory traj_left2 = null;
    Trajectory traj_left3 = null;
    Trajectory traj_left4 = null;
    Trajectory traj_left104 = null;
    Trajectory traj_left5 = null;
    Trajectory traj_left6 = null;
    Trajectory traj_left7 = null;
    Trajectory traj_left8 = null;


    // Miscellaneous
    private ElapsedTime runtime = new ElapsedTime();
    public int prevLine = -1;
    public int line = 0;

    @Override
    public void init() {

        // Declare multiple telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Declare drive
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        telemetry.update();
        // AUTONOMOUS TRAJECTORIES
        // Starting position
        drive.setPoseEstimate(START_POSE);

//        IGNORE
//                .lineToConstantHeading(new Vector2d(-18, -55))
//                .splineToSplineHeading(new Pose2d(-10, -17.5, Math.toRadians(169)), Math.toRadians(90))
//        IGNORE

        // Prop on the left
        traj_left1 = drive.trajectoryBuilder(START_POSE)
                .lineToLinearHeading(new Pose2d(33.2, 32, Math.toRadians(180)))
                .build(); // spike mark

        traj_left2 = drive.trajectoryBuilder(traj_left1.end())
                .lineToLinearHeading(new Pose2d(49, 41, Math.toRadians(180)))
                .build(); // backdrop

        traj_left3 = drive.trajectoryBuilder(traj_left2.end())
                .lineToLinearHeading(new Pose2d(34, 40, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(-51, 20, Math.toRadians(180)), Math.toRadians(180))
                .build(); // line up to stack

        traj_left4 = drive.trajectoryBuilder(traj_left3.end())
                .lineToLinearHeading(new Pose2d(28, 20, Math.toRadians(180)))
                .splineToSplineHeading(new Pose2d(50, 36, Math.toRadians(180)), Math.toRadians(180))
                .build(); // go backdrop

        traj_left5 = drive.trajectoryBuilder(traj_left4.end())
                .lineToConstantHeading(new Vector2d(38, 55))
                .build();

        traj_left6 = drive.trajectoryBuilder(traj_left5.end())
                .lineToConstantHeading(new Vector2d(61, 60))
                .build();

        // Build Autonomous Program
        buildProgram();
    }

    @Override
    public void loop() {
        if (line < PROGRAM.size()) {
            int func = PROGRAM.get(line)[0];
            int arg1 = PROGRAM.get(line)[1];

            boolean CHANGE_LINE = false;

            switch (func) {
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
            }

            prevLine = line;
            if (CHANGE_LINE) {
                line += 1;
            }
        }

        drive.update();

        // TELEMETRY
        telemetry.addData("Line", line);
        telemetry.update();
    }

    // AUTONOMOUS FUNCTIONS:



    // Trajectory Functions
    public void followTraj(int trajNo) {
        PROGRAM.add(new int[] {3, trajNo, 0, 0});
    }

    public void waitTime(int ms) {
        PROGRAM.add(new int[] {5, ms, 0, 0});
    }
    public void waitTrajDone() {
        PROGRAM.add(new int[] {7, 0, 0, 0});
    }

    public void buildProgram() {
        // Trajectory 1
        followTraj(1);
        waitTrajDone();

        // Trajectory 2
        followTraj(2);
        waitTrajDone();

        // Trajectory 3
        followTraj(3);
        waitTrajDone();

        // Trajectory 4
        followTraj(4);
        waitTrajDone();

        // Trajectory 5
        followTraj(5);
        waitTrajDone();

        // Trajectory 6
        followTraj(6);
        waitTrajDone();

        // Trajectory 7
        followTraj(7);
        waitTrajDone();

        // Trajectory 8
        followTraj(8);
        waitTrajDone();
    }
}
