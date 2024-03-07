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
public class AO_BlueBackdropMiddle extends OpMode {

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
            .splineToLinearHeading(new Pose2d(24, 24, Math.toRadians(-90)), Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(12, 24, Math.toRadians(-90)), Math.toRadians(0))
            .setReversed(true)
            .back(24)
            .setReversed(false)
            .splineToLinearHeading(new Pose2d(48, 36, Math.toRadians(0)), Math.toRadians(0))
            .back(108)
            .forward(108)
            .back(108)
            .forward(108)
            .strafeLeft(24)
            .forward(12)
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
