package org.firstinspires.ftc.teamcode.JoshAuton;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous
public class Auton_BlueWingPlusBackdrop extends OpMode {

    RobotMethods robot = new RobotMethods();

    // tfod
    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "OldPropModel.tflite";
    private static final String[] LABELS = { "Blue Marker", "Red Marker" };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private String pos = "right";

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
    }

    @Override
    public void start() {
        double dist;
        switch (pos) {
            case "left":
                // drive to spike mark
                robot.drive(0, 28, 1);
                robot.returnAfterBusy();
                // mv on spike mark
                robot.drive(6, 0, 1);
                robot.returnAfterBusy();
                // move lift out of the way
                robot.moveLift(1000, 1, telemetry);
                robot.returnAfterBusy();
                // drop pixel
                robot.middle(0.5);
                sleep(1000);
                robot.middle(0);
                // move lift back down
                robot.moveLift(-1000, 1, telemetry);
                // drive towards backdrop (thru trusses)
                robot.drive(-12, 0, 1);
                robot.returnAfterBusy();
                robot.drive(0, 24, 1);
                robot.returnAfterBusy();
                robot.drive(84, 0, 1);
                robot.returnAfterBusy();
                // move lift up
                robot.moveLift(1000, 1, telemetry);
                // drive towards left drop pos
                robot.drive(0, -32, 1);
                robot.returnAfterBusy();
                // drive towards backdrop at 20% speed
                robot.drive(10, 0, 0.2);
                dist = robot.getBackdropDistance();
                // wait until robot is less than 3.5 inches away from the backdrop
                while (dist > 3.5) {
                    dist = robot.getBackdropDistance();
                }
                // stop the wheels
                robot.stopWheels();
                // drop pixel
                robot.setDropServo(.5);
                sleep(1000);
                // back away from the backdrop a bit
                robot.drive(-2, 0, 0.5);
                robot.returnAfterBusy();
                robot.setDropServo(0.045);
                break;
            case "middle":
                // drive to prop
                robot.drive(0, 32, 1);
                robot.returnAfterBusy();
                robot.turn(90, 1);
                robot.returnAfterBusy();
                // move the lift out of the way
                robot.moveLift(1000, 1, telemetry);
                robot.returnAfterBusy();
                // drop pixel
                robot.middle(1);
                sleep(1000);
                robot.middle(0);
                // move the lift down
                robot.moveLift(-1000, 1, telemetry);
                // drive towards backdrop (thru trusses)
                robot.drive(-4, 0, 1);
                robot.returnAfterBusy();
                robot.turn(-90, 1);
                robot.returnAfterBusy();
                robot.drive(76, 0, 1);
                robot.returnAfterBusy();
                // move the lift up
                robot.moveLift(1000, 1, telemetry);
                // drive towards middle drop pos
                robot.drive(0, 4, 1);
                robot.returnAfterBusy();
                // drice towards backdrop at 20% speed
                robot.drive(10, 0, 0.2);
                dist = robot.getBackdropDistance();
                // wait until the robot is less than 3.5 inches away from the backdrop
                while (dist > 3.5) {
                    dist = robot.getBackdropDistance();
                }
                // stop the wheels
                robot.stopWheels();
                // drop pixel
                robot.setDropServo(.5);
                sleep(1000);
                // back up a bit from the backdrop
                robot.drive(-2, 0, 0.5);
                robot.returnAfterBusy();
                robot.setDropServo(0.045);
                break;
            case "right":
                // drive to prop
                robot.drive(0, 28, 1);
                robot.returnAfterBusy();
                robot.turn(180, 1);
                robot.returnAfterBusy();
                robot.drive(6, 0, 1);
                robot.returnAfterBusy();
                // move lift out of the way
                robot.moveLift(1000, 1, telemetry);
                robot.returnAfterBusy();
                // drop prop
                robot.middle(0.5);
                sleep(1000);
                // move lift down
                robot.moveLift(-1000, 0, telemetry);
                robot.middle(0);
                // drive towards backdrop
                robot.drive(-76, 0, 1);
                robot.returnAfterBusy();
                // move lift up
                robot.moveLift(1000, 1, telemetry);
                // drive towards backdrop
                robot.turn(180, 1);
                robot.returnAfterBusy();
                robot.drive(6, 0, 1);
                robot.returnAfterBusy();
                robot.drive(0, 6, 1);
                robot.returnAfterBusy();
                // drive towards backdrop at 20% speed
                robot.drive(10, 0, 0.2);
                dist = robot.getBackdropDistance();
                // wait until robot is less than 3.5 inches away from backdrop
                while (dist > 3.5) {
                    dist = robot.getBackdropDistance();
                }
                // stop the wheels
                robot.stopWheels();
                // drop pixel
                robot.setDropServo(.5);
                sleep(1000);
                // back up a bit from the backdrop
                robot.drive(-2, 0, 0.5);
                robot.returnAfterBusy();
                robot.setDropServo(0.045);
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

    private String detectProp() {

        String pos = "right";
        List<Recognition> currentRecognitions = tfod.getRecognitions();

        if (currentRecognitions.size() > 0) {
            Recognition recognition = currentRecognitions.get(0);

            double x = (recognition.getLeft() + recognition.getRight()) / 2;
            if (x < 300) {
                telemetry.addLine("Spike Mark: left");
                pos = "left";
            } else {
                telemetry.addLine("Spike Mark: middle");
                pos = "middle";
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
            if (recognition.getLabel().equals(autonColor) && recognition.getConfidence() > 0.6) {
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

    }


}
