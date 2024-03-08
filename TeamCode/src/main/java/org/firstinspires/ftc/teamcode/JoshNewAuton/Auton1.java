package org.firstinspires.ftc.teamcode.JoshNewAuton;

import android.util.Size;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

    BaseMethods base = new BaseMethods(telemetry, robot);

    // tfod
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "OldPropModel.tflite";
    private static final String[] LABELS = { "Blue Marker", "Red Marker" };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;


    private String pos = "right";

    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


    //trajectory

    //// LEFT TRAJECTORIES
    /// LEFT TRAJECTORY 1
    TrajectorySequence left1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
            // DRIVE TO SPIKE MARK
            .splineToSplineHeading(new Pose2d(31, 30, Math.toRadians(180)), Math.toRadians(180))
            .build();
    TrajectorySequence left2 = drive.trajectorySequenceBuilder(left1.end())
            // REVERSE
            .setReversed(true)
            // DRIVE TO BACKDROP
            .lineToSplineHeading(new Pose2d(50, 42, Math.toRadians(0)))
            .build();
    TrajectorySequence left3 = drive.trajectorySequenceBuilder(left2.end())
            // PARK ON LEFT
            .strafeLeft(18)
            .forward(10)
            .build();

    TrajectorySequence middle1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
            // DRIVE TO SPIKE MARK
            .splineToSplineHeading(new Pose2d(12, 31, Math.toRadians(270)), Math.toRadians(180))
            .build();
    TrajectorySequence middle2 = drive.trajectorySequenceBuilder(middle1.end())
            // REVERSE
            .setReversed(true)
            // DRIVE TO BACKDROP
            .lineToSplineHeading(new Pose2d(50, 36, Math.toRadians(0)))
            .build();
    TrajectorySequence middle3 = drive.trajectorySequenceBuilder(middle2.end())
            // PARK IN CENTER
            .strafeLeft(24)
            .forward(10)
            .build();

    TrajectorySequence right1 = drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
            // DRIVE TO SPIKE MARK
            .splineToSplineHeading(new Pose2d(7, 36, Math.toRadians(180)), Math.toRadians(180))
            .build();
    TrajectorySequence right2 = drive.trajectorySequenceBuilder(right1.end())
            // REVERSE
            .setReversed(true)
            .lineToSplineHeading(new Pose2d(50, 30, Math.toRadians(0)))
            .build();
    TrajectorySequence right3 = drive.trajectorySequenceBuilder(right2.end())
            // PARK ON RIGHT
            .strafeLeft(30)
            .forward(10)
            .build();
    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);


        initTfod();

        int i = 0;
        while (i < 300 && pos.equals("right")) {
            pos = detectProp("Blue Marker");
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
                drive.followTrajectorySequence(left1);

                robot.moveLift(400, 1);
                sleep(500);
                robot.setDropServo(0.5);
                sleep(500);
                robot.setDropServo(0.045);
                sleep(200);
                robot.moveLift(-400, 1);

                drive.followTrajectorySequence(left2);

                robot.moveLift(400, 1);
                sleep(500);
                robot.middle(0.5);
                sleep(500);
                robot.middle(0);
                robot.moveLift(-400, 1);

                drive.followTrajectorySequence(left3);
                break;
            case "middle":
                drive.followTrajectorySequence(middle1);

                robot.moveLift(400, 1);
                sleep(500);
                robot.setDropServo(0.5);
                sleep(500);
                robot.setDropServo(0.045);
                sleep(200);
                robot.moveLift(-400, 1);

                drive.followTrajectorySequence(middle2);

                robot.moveLift(400, 1);
                sleep(500);
                robot.middle(0.5);
                sleep(500);
                robot.middle(0);
                robot.moveLift(-400, 1);

                drive.followTrajectorySequence(middle3);
                break;
            case "right":
                drive.followTrajectorySequence(right1);

                robot.moveLift(400, 1);
                sleep(500);
                robot.setDropServo(0.5);
                sleep(500);
                robot.setDropServo(0.045);
                sleep(200);
                robot.moveLift(-400, 1);

                drive.followTrajectorySequence(right2);

                robot.moveLift(400, 1);
                sleep(500);
                robot.middle(0.5);
                sleep(500);
                robot.middle(0);
                robot.moveLift(-400, 1);
                
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

    private void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                .setModelInputSize(300)
                .setModelAspectRatio(1.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.4f);

        // Disable or re-enable the TFOD processor at any time.
        visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    private String detectProp(String autonColor) {
        // set default pos to right
        String pos = "right";
        // get list of all recognitions
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        // if there are any recognitions
        if (currentRecognitions.size() > 0) {
            // get the first recognition
            Recognition recognition = currentRecognitions.get(0);

            // check if the recognition label matches the autonomous color and its confidence is above 70 percent
            if (recognition.getLabel().equals(autonColor) && recognition.getConfidence() > 0.7) {
                // get the x position of the recognition
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                // if the x position is less than 300 (on the left)
                if (x < 300) {
                    // set the pos to left
                    telemetry.addLine("Spike Mark: left");
                    pos = "left";
                } else { // if the x position is more than 300 (on the right)
                    // set the pos to middle (bc the camera can only see left and middle spike marks)
                    telemetry.addLine("Spike Mark: middle");
                    pos = "middle";
                }
            }
        }

        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        return pos;

    }   // end method telemetryTfod()


}
