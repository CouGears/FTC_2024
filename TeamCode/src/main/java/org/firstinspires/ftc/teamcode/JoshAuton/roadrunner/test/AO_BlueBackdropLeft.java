package org.firstinspires.ftc.teamcode.JoshAuton.roadrunner.test;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.JoshAuton.RobotMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class AO_BlueBackdropLeft extends OpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
            .splineToConstantHeading(new Vector2d(24, 30), Math.toRadians(0))
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(18, 56, Math.toRadians(-90)), Math.toRadians(0))
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(48, 40, Math.toRadians(0)), Math.toRadians(-90))
            .strafeRight(12)
            .setReversed(true)
            .splineToLinearHeading(new Pose2d(24, -12, Math.toRadians(270)), Math.toRadians(90))
            .strafeRight(12)
            .splineToLinearHeading(new Pose2d(12, 36, Math.toRadians(0)), Math.toRadians(90))
            .back(72)
            .strafeLeft(24)
            .setReversed(false)
            .forward(120)
            .build();

    @Override
    public void init() {
    }

    @Override
    public void start() {
        drive.followTrajectorySequence(traj);
    }

    @Override
    public void loop() {
    }

    public void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            telemetry.addLine("Failed Sleep");
            telemetry.update();
        }
    }
}
