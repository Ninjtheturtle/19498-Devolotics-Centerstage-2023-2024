package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp

public class Values extends OpMode {
    private Servo intakeL, intakeR, lockFront, lockBack, vPitchL, vPitchR, launch, purpPixel, pivot;
    //vPitch servo positions
    public static double vPitchIntake = 1;
    public static double vPitchDeposit = 0.09;

    //intake servo positions
    public static double intakeDown = 0.33;
    public static double intakeUp = 0.08;

    //locking servo positions
    public static double lockUpFront = 0.25;
    public static double lockDownFront = 0;

    public static double lockDownBack = 0;
    public static double lockUpBack = 0.25;

    //pivot servo positions
    public static double pivotHome = 0.14;
    public static double pivotScore = 0.949;

    //Setting servo variables
    public static double intakePos = intakeDown;
    public static double vPitchPos = vPitchIntake;
    public static double pivotPos = pivotHome;
    public static double lockAngleFront = 0;
    public static double lockAngleBack = 0;

    @Override
    public void init() {
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

        pivot = hardwareMap.get(Servo.class, "s12");
        pivot.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        pivot.setPosition(pivotPos);
        lockBack.setPosition(lockAngleBack);
        lockFront.setPosition(lockAngleFront);

        intakeL.setPosition(intakePos - 0.02);
        intakeR.setPosition(intakePos);

        vPitchL.setPosition(vPitchPos);
        vPitchR.setPosition(vPitchPos - 0.02);

        telemetry.addData("Pivot Position", pivotPos);
        telemetry.addData("vPitchPos", vPitchPos);
        telemetry.addData("Lock Back", lockAngleBack);
        telemetry.addData("Lock Front", lockAngleFront);
        telemetry.addData("Intake Position", intakePos);
        telemetry.addData("vPitchPos", vPitchPos);
    }
}
