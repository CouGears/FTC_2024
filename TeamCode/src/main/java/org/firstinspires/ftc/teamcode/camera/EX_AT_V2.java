/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.camera;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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

@TeleOp
@Disabled
//******************WORKS!******************

public class EX_AT_V2 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.
    final double DESIRED_DISTANCE = 15.0; //  this is how close the camera should get to the target (inches)
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    AutonMethods robot = new AutonMethods();
    private boolean end = false;


    @Override public void runOpMode() {

        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)


        // Initialize the Apriltag Detection process
        initAprilTag();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorFL");  //  Used to control the left front drive wheel
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorFR");  //  Used to control the right front drive wheel
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorBL");  //  Used to control the left back drive wheel
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorBR");  //  Used to control the right back drive wheel

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips


        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            switch (robot.counter) {
                case 0: //natural state
                    DESIRED_TAG_ID = 0;
                    end = false;
                    targetFound = false;
                    desiredTag = null;
                    while (!gamepad1.left_bumper) { //press left bumper when tag is found

                        // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                        leftFrontDrive.setPower(((this.gamepad1.right_stick_x) - (this.gamepad1.right_stick_y) + ((this.gamepad1.left_stick_x)) - (this.gamepad1.left_stick_y)) * .5);
                        rightFrontDrive.setPower(-((this.gamepad1.right_stick_x) + (this.gamepad1.right_stick_y) + (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y)) * .5);
                        leftBackDrive.setPower(-(-(this.gamepad1.right_stick_x) + (this.gamepad1.right_stick_y) - (this.gamepad1.left_stick_x) - (this.gamepad1.left_stick_y)) * .5);
                        rightBackDrive.setPower((-(this.gamepad1.right_stick_x) - (this.gamepad1.right_stick_y) - (this.gamepad1.left_stick_x) + (this.gamepad1.left_stick_y)) * .5);

                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                        for (AprilTagDetection detection : currentDetections) { // Step through the list of detected tags and look for a matching tag
                            if ((detection.metadata != null) &&
                                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                                targetFound = true;
                                desiredTag = detection;
                                break;  // don't look any further.
                            }
                        }

                        // Tell the driver what we see, and what to do.
                        if (targetFound) {
                            telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                            telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                            telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                            telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
                        } else {
                            telemetry.addData(">", "Drive using joysticks to find valid target\n");
                        }

                        telemetry.update();
                    }
                    robot.counter++;
                    break;

                case 1:
                    while (DESIRED_TAG_ID == -1) {
                        if (gamepad1.a) {
                            DESIRED_TAG_ID = 1;
                        } else if (gamepad1.b) {
                            DESIRED_TAG_ID = 2;
                        } else if (gamepad1.y) {
                            DESIRED_TAG_ID = 3;
                        }
                        telemetry.addData("Please press button:", "");
                        telemetry.addData("A =:", "1");
                        telemetry.addData("B =:", "2");
                        telemetry.addData("Y =:", "3");
                        telemetry.update();
                    }
                    telemetry.clearAll();
                    robot.counter++;
                    break;

                case 2:
                    while (desiredTag.ftcPose.range >= DESIRED_DISTANCE && !end) {

                        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                        for (AprilTagDetection detection : currentDetections) { // Step through the list of detected tags and look for a matching tag
                            if ((detection.metadata != null) &&
                                    ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                                desiredTag = detection;
                                break;  // don't look any further.
                            }
                        }

                        double rangeError = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                        double headingError = desiredTag.ftcPose.bearing;
                        double yawError = desiredTag.ftcPose.yaw;

                        // Use the speed and turn "gains" to calculate how we want the robot to move.
                        drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                        turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                        strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                        telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                        telemetry.addData("Dist", "%3f", desiredTag.ftcPose.range);
                        telemetry.update();

                        // Apply desired axes motions to the drivetrain.
                        moveRobot(drive, strafe, turn);
                        sleep(10);

                        if (desiredTag.ftcPose.range <= (DESIRED_DISTANCE + 1.5)) {
                            end = true;
                        }

                    }
                    telemetry.addData("DONE", "");
                    telemetry.update();
                    robot.counter = 0;
                    break;
            }
        }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
    }

    /**
     * Move robot according to desired axes motions
     * <p>
     * Positive X is forward
     * <p>
     * Positive Y is strafe left
     * <p>
     * Positive Yaw is counter-clockwise
     */
    public void moveRobot(double x, double y, double yaw) {

        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    /**
     * Initialize the AprilTag processor.
     */
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

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
