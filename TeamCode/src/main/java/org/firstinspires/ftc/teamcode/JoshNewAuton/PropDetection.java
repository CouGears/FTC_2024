package org.firstinspires.ftc.teamcode.JoshNewAuton;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.Objects;

public class PropDetection {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "RoundedCubeModel.tflite";
    private static final String[] LABELS = { "Blue Marker", "Red Marker" };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    private String color;

    private String markerLabel;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public PropDetection(HardwareMap hardwareMap, Telemetry telemetry, String color) {
        this.color = color;
        switch (color) {
            case "Blue":
                this.markerLabel = "Blue Marker";
                break;
            case "Red":
                this.markerLabel = "Red Marker";
        }
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public PropDetection(HardwareMap hardwareMap, Telemetry telemetry) {
        this.color = "N/A";
        this.markerLabel = "N/A";
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
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
    }

    public String scan() {
        // set default pos to right
        String pos = "right";
        // get list of all recognitions
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        // if there are any recognitions
        if (currentRecognitions.size() > 0) {
            // get the first recognition
            Recognition recognition = currentRecognitions.get(0);

            // get all recognitions
            for (Recognition rec : currentRecognitions) {
                // if tensorflow is more confident about another recognition
                if (rec.getConfidence() > recognition.getConfidence()) {
                    // use that recognition
                    recognition = rec;
                }
            }

            // check if the recognition label matches the autonomous color and its confidence is above 40 percent
            if ((recognition.getLabel().equals(markerLabel) || Objects.equals(markerLabel, "N/A")) && recognition.getConfidence() >= 0.4) {
                telemetry.addData("label", recognition.getLabel());
                telemetry.addData("marker label", markerLabel);
                telemetry.update();
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
