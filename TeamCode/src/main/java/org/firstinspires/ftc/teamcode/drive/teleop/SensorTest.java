package org.firstinspires.ftc.teamcode.drive.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp
//@Config
public class SensorTest extends OpMode
{
    private SampleMecanumDrive drive;
    DistanceSensor senseLF, senseLB;

    double prevTime = 0;
    double prevTimeTelemetry = 0;
    private ElapsedTime totalTime = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        senseLF = hardwareMap.get(DistanceSensor.class, "senseLF");
//        senseLB = hardwareMap.get(DistanceSensor.class, "senseLB");
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x
                )
        );
        drive.update();

        double overallTime;
        overallTime = totalTime.milliseconds() - prevTime;
        prevTime = totalTime.milliseconds();

//        double pee1 = senseLF.getDistance(DistanceUnit.MM);
//        double pee2 = senseLB.getDistance(DistanceUnit.MM);

        if (totalTime.milliseconds() - prevTimeTelemetry > 500) {
            telemetry.addData("Loop times", overallTime);
//            telemetry.addData("Front Lock", pee1);
//            telemetry.addData("Back Lock", pee2);
            telemetry.update();
            prevTimeTelemetry = totalTime.milliseconds();
        }

    }
}