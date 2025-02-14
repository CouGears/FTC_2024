package org.firstinspires.ftc.teamcode.JoshNewAuton;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.JoshAuton.RobotMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class Auton7 extends OpMode {

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
                // PARK IN CENTER
                .strafeRight(6)
                .build();

        middle1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
                // DRIVE TO SPIKE MARK
                .strafeRight(6)
                .splineToSplineHeading(new Pose2d(12, 33, Math.toRadians(270)), Math.toRadians(180))
                .build();
        middle2 = drive.trajectorySequenceBuilder(middle1.end())
                .back(4)
                // REVERSE
                .setReversed(true)
                // DRIVE TO BACKDROP
                .lineToSplineHeading(new Pose2d(50, 36, Math.toRadians(0)))
                .build();
        middle3 = drive.trajectorySequenceBuilder(middle2.end())
                // PARK IN CENTER
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
                // PARK IN CENTER
                .strafeLeft(6)
                .build();

        Pose2d startPose = new Pose2d(12, 64.5, 0);
        drive.setPoseEstimate(startPose);

        int i = 0;
        while (i < 300 && pos.equals("right")) {
            pos = scanner.scan();
            telemetry.update();
            sleep(10);
            i++;
        }

    }

    @Override
    public void start() {

        double dist;
        telemetry.addData("pos", pos);
        telemetry.update();
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
