package org.firstinspires.ftc.teamcode.drive.teleop;

public class Values {
    // vPitch servo positions
    public static final double vPitchIntake = 0.91;
    public static final double vPitchDeposit = 0.04;

    // INTAKE SERVO POSITIONS
    public static final double intakeDown = 0.45;
    public static final double intakeUp = 0.1;

    // PIVOT SERVO POSITIONS
    public static final double pivotHome = 0.13;
    public static final double pivotScore = 0.84;

    // LOCKING SERVO POSITIONS
    public static final double lockUpFront = 0;
    public static final double lockUpBack = 0;
    public static final double lockDownFront = 0.25;
    public static final double lockDownBack = 0.25;

    public static final double intakeKickOut = 0;
    public static final double kickoutSpeed = -1;

    // SLIDE PIDS
    public static final double Pv = 0.010877, Iv = 0, Dv = 0.00006, Fv = 0;

    // DEPOSIT POSITIONS
    public static final int targetLow = 500;
    public static final int targetMed = 600;
    public static final int targetHigh = 700;
    public static final int vMin = 0;
    public static final int vIntake = 40;
    public static final int vTargetInter = 70;
    public static final int vMax = 1300;

    // DRONE SERVO POSITIONS
    public static final double launchReleased = 0.2;
    public static final double launchArmed = 0.2;
}