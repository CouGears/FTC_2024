package org.firstinspires.ftc.teamcode.JoshAuton.roadrunner.test;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.robot.Robot;

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
public class AO_BlueBackdropRight extends OpMode {

    SampleMecanumDrive drive;
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    RobotMethods robot = new RobotMethods();
    Pose2d startPose = new Pose2d(12, 64.5, Math.toRadians(0));



    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        drive = new SampleMecanumDrive(hardwareMap);
        traj1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(48, 30))
                .build();


        traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(7, 36, Math.toRadians(180)))

                .build();

        traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .setReversed(true)
                .back(12)
                .splineToSplineHeading(new Pose2d(12, 60, Math.toRadians(0)), Math.toRadians(180))
                .back(48)
                .splineToConstantHeading(new Vector2d(-57, 36), Math.toRadians(180))
                .build();
    }

    @Override
    public void start() {
        drive.setPoseEstimate(startPose);

        drive.followTrajectorySequence(traj1);

        robot.moveLift(400, 1, telemetry);
        sleep(500);
        robot.setDropServo(0.5);
        sleep(500);
        robot.setDropServo(0.045);
        sleep(200);
        robot.moveLift(-400, 1, telemetry);

        drive.followTrajectorySequence(traj2);

        robot.moveLift(400, 1, telemetry);
        sleep(500);
        robot.middle(0.5);
        sleep(500);
        robot.middle(0);
        robot.moveLift(-400, 1, telemetry);

        drive.followTrajectorySequence(traj3);

        robot.setIntakeString(1);
        sleep(2500);
        robot.setIntakeString(0);
        robot.backIntake(-1);
        robot.middle(0.5);
        sleep(500);
        robot.middle(0);
        robot.setIntakeString(1);
        sleep(500);
        robot.setIntakeString(0);
        sleep(500);
        robot.backIntake(0);

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
