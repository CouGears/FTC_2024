package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.AutonMethods;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous
@Disabled

public class AprilTagAutoV1 extends OpMode {

    //******************OUR VARS******************
    private static final double rev = 537.7;
    private static final double inch = rev / (3.78 * 3.14);
    private static final double feet = inch * 12 + (10 * inch);
    private ElapsedTime runtime = new ElapsedTime();
    AutonMethods robot = new AutonMethods();
    HardwareMap map;
    Telemetry tele;
    boolean on = false;
    //******************FTC EXAMPLE VARS******************
    boolean targetFound     = false;    // Set to true when an AprilTag target is detected
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    public void init() {
        robot.initBasic(hardwareMap, telemetry);
        initAprilTag();
        if (USE_WEBCAM) setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void start() {
        runtime.reset();
    }

    public void loop() {
        on = true;
        switch (robot.counter) {
            case 0://Finding Tag

                telemetry.addData("Step", "Case 0");
                telemetry.update();

                int totalTurned = 0;
                while (targetFound == false) {
                    robot.turn(20);
                    totalTurned = totalTurned + 20;
                    telemetry.addData("Case", "0");
                    telemetry.addData("Ive turned", "%3 degrees", totalTurned);
                    telemetry.update();
                    List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                    for (AprilTagDetection detection : currentDetections) {
                        if ((detection.metadata != null) && ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                            targetFound = true;
                            desiredTag = detection;
                        }
                    }
                    if (totalTurned > 360){
                        telemetry.addData("ERROR", "No target found");
                        telemetry.update();
                        break;
                    }
                }
                telemetry.addData("TARGET FOUND!" , "");
                telemetry.addData("target", desiredTag);
                telemetry.update();
                robot.counter++;
                break;
            case 1: //Turning to tag
                telemetry.addData("Step", "Case 1");
                telemetry.update();
                robot.sleep(1000);
                double bearingInRads =  Math.toRadians(desiredTag.ftcPose.bearing);
                double strafeDistance = Math.cos(bearingInRads) * desiredTag.ftcPose.range;
                telemetry.addData("STRAFE CACULATED!" , "");
                telemetry.addData("strafeDistance", strafeDistance);
                telemetry.update();
                robot.sleep(5000);
                for (int i = 0; i < 4; i++) { //strafing
                    robot.drive(0, strafeDistance/4, .5);
                    robot.turn(desiredTag.ftcPose.bearing);
                }
                robot.counter++;

                break;
            case 2: //
                telemetry.addData("Step", "Case 2");
                telemetry.update();
                robot.sleep(5000);
                robot.drive( desiredTag.ftcPose.range - 16, 0,1);
                robot.counter++;
                break;
            case 3:
                break;
        }
    }


    //**********************FTC EXAMPLE APRILTAG FUNCTIONS**********************
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Ready");
            telemetry.update();
            while (on = false && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                robot.sleep(20);
            }
        }

        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
            exposureControl.setMode(ExposureControl.Mode.Manual);
        }
        exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
        gainControl.setGain(gain);
    }

    private void scanAprilTag(){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        // Tell the driver what we see, and what to do.
        if (targetFound) {
            telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
            telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
            telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
        } else {
            telemetry.addData(">","Turning\n");
        }
    }
}


