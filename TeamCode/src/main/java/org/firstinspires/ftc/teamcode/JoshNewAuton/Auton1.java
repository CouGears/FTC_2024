package org.firstinspires.ftc.teamcode.JoshNewAuton;

import android.util.Size;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.fasterxml.jackson.databind.annotation.JsonAppend;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
public class Auton1 extends OpMode {

    // initialize new instance of robot
    RobotMethods robot = new RobotMethods();

    BaseMethods base = new BaseMethods(telemetry, robot, hardwareMap);

    private String pos = "right";

    PropDetection scanner = new PropDetection(hardwareMap, telemetry, "Blue");

    TrajectorySequence left1;
    TrajectorySequence left2;
    TrajectorySequence left3;
    TrajectorySequence middle1;
    TrajectorySequence middle2;
    TrajectorySequence middle3;
    TrajectorySequence right1;
    TrajectorySequence right2;
    TrajectorySequence right3;

    SampleMecanumDrive drive;

    @Override
    public void init() {
        base.init(hardwareMap, telemetry);

        scanner.init(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);

        //trajectory

        //// LEFT TRAJECTORIES
        /// LEFT TRAJECTORY 1
        left1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
                // DRIVE TO SPIKE MARK
                .splineToSplineHeading(new Pose2d(31, 30, Math.toRadians(180)), Math.toRadians(180))
                .build();
        left2 = drive.trajectorySequenceBuilder(left1.end())
                // BACK AWAY FROM PURPLE PIXEL
                .back(6)
                // REVERSE
                .setReversed(true)
                // DRIVE TO BACKDROP
                .lineToSplineHeading(new Pose2d(50, 42, Math.toRadians(0)))
                .build();
        left3 = drive.trajectorySequenceBuilder(left2.end())
                // PARK ON LEFT
                .strafeLeft(18)
                .forward(10)
                .build();

        middle1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
                // DRIVE TO SPIKE MARK
                .splineToSplineHeading(new Pose2d(12, 31, Math.toRadians(270)), Math.toRadians(180))
                .build();
        middle2 = drive.trajectorySequenceBuilder(middle1.end())
                // REVERSE
                .setReversed(true)
                // DRIVE TO BACKDROP
                .lineToSplineHeading(new Pose2d(50, 36, Math.toRadians(0)))
                .build();
        middle3 = drive.trajectorySequenceBuilder(middle2.end())
                // PARK IN CENTER
                .strafeLeft(24)
                .forward(10)
                .build();

        right1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
                // DRIVE TO SPIKE MARK
                .splineToSplineHeading(new Pose2d(7, 36, Math.toRadians(180)), Math.toRadians(180))
                .build();
        right2 = drive.trajectorySequenceBuilder(right1.end())
                // REVERSE
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(50, 30, Math.toRadians(0)))
                .build();
        right3 = drive.trajectorySequenceBuilder(right2.end())
                // PARK ON RIGHT
                .strafeLeft(30)
                .forward(10)
                .build();

        int i = 0;
        while (i < 300 && pos.equals("right")) {
            pos = scanner.scan();
            telemetry.update();
            sleep(10);
            i++;
        }

        Pose2d startPose = new Pose2d(12, 64.5, 0);
        drive.setPoseEstimate(startPose);
    }

    @Override
    public void start() {

        double dist;
        switch (pos) {
            case "left":
                // drive to spike mark
                drive.followTrajectorySequence(left1);
                // spit pixel
                base.spitPixel();
                // drive to backdrop
                drive.followTrajectorySequence(left2);
                // place pixel
                base.placePixel();
                // park
                drive.followTrajectorySequence(left3);
                break;
            case "middle":
                // drive to spike mark
                drive.followTrajectorySequence(middle1);
                // spit pixel
                base.spitPixel();
                // drive to backdrop
                drive.followTrajectorySequence(middle2);
                // place pixel
                base.placePixel();
                // park
                drive.followTrajectorySequence(middle3);
                break;
            case "right":
                // drive to spike mark
                drive.followTrajectorySequence(right1);
                // spit pixel
                base.spitPixel();
                // drive to backdrop
                drive.followTrajectorySequence(right2);
                // place pixel
                base.placePixel();
                // park
                drive.followTrajectorySequence(right3);
                break;
        }
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
