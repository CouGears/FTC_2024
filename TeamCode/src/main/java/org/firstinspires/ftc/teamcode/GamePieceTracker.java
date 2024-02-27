package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

@Autonomous(name="GamePieceTrackerAuton")
public class GamePieceTracker extends LinearOpMode {

    private static final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_ASSET = "OldPropModel.tflite";
    private static final String[] LABELS = { "Blue Marker", "Red Marker" };

    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        initTfod();
        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                detectGamePiece();
                sleep(20);
            }
        }

        visionPortal.close();
    }

    private void initTfod() {
        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }

    private void detectGamePiece() {
        List<Recognition> recognitions = tfod.getRecognitions();

        if (recognitions.size() == 0) {
            telemetry.addData("Position", "Left");
        } else {
            for (Recognition recognition : recognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                String position = x > 440 ? "Right" : "Middle";
                String label = recognition.getLabel();
                telemetry.addData("Position", position);
                telemetry.addData("Color", label);
            }
        }
        telemetry.update();
    }
}
