package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@TeleOp(name="ColorAnalysisTeleOp", group="TeleOp")
@Disabled
public class ColorAnalysisTeleOp extends LinearOpMode {

    OpenCvCamera webcam;
    Bitmap bitmap;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();

        webcam.setPipeline(new ColorPipeLine());

        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_button) {
                analyzeImage(bitmap, "blue");
            } else if (gamepad1.right_stick_button) {
                analyzeImage(bitmap, "red");
            }
        }
    }

    public void analyzeImage(Bitmap image, String color) {
        int width = image.getWidth();
        int height = image.getHeight();

        int thirdWidth = width / 3;

        int leftColorSum = 0;
        int middleColorSum = 0;
        int rightColorSum = 0;

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int pixel = image.getPixel(x, y);

                if (color.equals("red")) {
                    int redValue = Color.red(pixel);

                    if (x < thirdWidth) {
                        leftColorSum += redValue;
                    } else if (x < 2 * thirdWidth) {
                        middleColorSum += redValue;
                    } else {
                        rightColorSum += redValue;
                    }
                } else if (color.equals("blue")) {
                    int blueValue = Color.blue(pixel);

                    if (x < thirdWidth) {
                        leftColorSum += blueValue;
                    } else if (x < 2 * thirdWidth) {
                        middleColorSum += blueValue;
                    } else {
                        rightColorSum += blueValue;
                    }
                }
            }
        }

        int leftAverage = leftColorSum / (thirdWidth * height);
        int middleAverage = middleColorSum / (thirdWidth * height);
        int rightAverage = rightColorSum / (thirdWidth * height);

        if (leftAverage > middleAverage && leftAverage > rightAverage) {
            telemetry.addData("Most " + color + " part", "left");
        } else if (middleAverage > leftAverage && middleAverage > rightAverage) {
            telemetry.addData("Most " + color + " part", "middle");
        } else {
            telemetry.addData("Most " + color + " part", "right");
        }

        telemetry.update();
    }

    class ColorPipeLine extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            bitmap = Bitmap.createBitmap(input.cols(), input.rows(), Bitmap.Config.ARGB_8888); // Access the bitmap field here
            Utils.matToBitmap(input, bitmap);
            return input;
        }
    }
}