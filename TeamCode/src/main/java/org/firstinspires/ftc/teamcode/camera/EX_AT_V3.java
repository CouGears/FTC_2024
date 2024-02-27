package org.firstinspires.ftc.teamcode.camera;

import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
@TeleOp
@Disabled
public class EX_AT_V3 extends LinearOpMode {
    //*****************************VARS*****************************
    //-----------------------------EXAMPLE VARS :(-----------------------------
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

    //-----------------------------NO TOUCH VARS >:(-----------------------------
    final double DESIRED_DISTANCE = 5.0; //  this is how close the camera should get to the target (inches)

    private DcMotor motorFL;  //  Used to control the left front drive wheel
    private DcMotor motorFR;  //  Used to control the right front drive wheel
    private DcMotor motorBL;  //  Used to control the left back drive wheel
    private DcMotor motorBR;  //  Used to control the right back drive wheel
    //
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //-----------------------------OUR VARS :D-----------------------------
    AutonMethods robot = new AutonMethods();
    public double FRtpos, BRtpos, FLtpos, BLtpos;
    double rev = 537.7; //312 rpm motor
    double inch = rev / (3.78 * 3.14);
    double feet = inch * 12 + (10 * inch);
    int attempt = 0;
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;

    @Override
    public void runOpMode() {

        //*****************************INIT***************************** (WORKS)
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        robot.initBasic(hardwareMap, telemetry);
        initAprilTag();
        hardwareSetup();
        setManualExposure(6, 250);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "ALL INIT DONE");
        telemetry.update();
        waitForStart();


        //*****************************OP-MODE*****************************
        //-----------------------------CHOOSE ATTEMPT----------------------------- (WORKS)
        while (opModeIsActive()) {
            while (attempt == 0){
                if (gamepad1.a){ attempt = 1;}
                telemetry.addData("a = ", "All at once (just AT data)");
                if (gamepad1.b){ attempt = 2;}
                telemetry.addData("b = ", "All at once (Trig)");
                if (gamepad1.y){ attempt = 3;}
                telemetry.addData("y = ", "Incremental");
                telemetry.update();
            }
            if (gamepad1.x){ attempt = 0;}
            telemetry.addData("x = ", "Reset attempt");

            //-----------------------------FIND AT----------------------------- (WORKS [* NO TOUCHY >:( *])
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null) &&
                        ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))) {
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                } else {
                    telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
                }
            }

            //-----------------------------BOT SEE AT----------------------------- (WORKS)
            if (targetFound) {
                telemetry.addData(">", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData(">", "Range %2f", desiredTag.ftcPose.range);
                telemetry.addData(">", "Bearing %2f", desiredTag.ftcPose.bearing);

            } else {
                telemetry.addData(">", "Drive using joysticks to find valid target\n");
            }

            //-----------------------------AUTON-----------------------------
            if (gamepad1.left_bumper && targetFound) {

                /*
                //MOVE TO TAG. NEED TO FIGURE OUT HOW TO DO THIS
                DIRECTLY USING robot.move
                MATH USING Math.cos & Math.toRad
                ITERATIVE WITH while (!linedUp) {moveL/moveR/moveFWD}
                 */


                //.............................ATTEMPT ALL AT ONCE............................. (RLY NOT CONFIDENT [ALL CODE IS FROM EXAMPLE})
                if (attempt == 1) {
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;

                    double  drive  = 0;// Desired forward power/speed (-1 to +1)
                    double  strafe = 0;// Desired strafe power/speed (-1 to +1)
                    double  turn   = 0;// Desired turning power/speed (-1 to +1)

                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                    moveRobot(drive, strafe, turn); //Telemetry is in this func

                }
                //.............................ATTEMPT MATH............................. (~KINDA~ CONFIDENT)
                if (attempt == 2) {
                    /*
                                    WHAT ROBOT SEES                EQUATIONS
                                         TAG                           C
                                       /     |                      /     |
                                      /      |                     /      |
                                     /       |                    /       |
                                    /        |                   /        |
                                   /         |                  /         |
                                  /          |                 /          |
                                 /           |                /           |
                          range /            |               /            |
                               /             |              /             |
                              /              |             /              | sqrt(AB^2 - AC^2)
                             /               |            /               |
                            /                |           /                |
                           /                 |          /                 |
                          /                  |         /                  |
                         /                   |        /                   |
                        /                    |       /                    |
                       /                     |      /                     |
                      /                      |     /                      |
                     /  bearing              |    /                       |
                    BOT______________________|   A ______________________ B
                                                       AC * cos(<BAC)
                     */

                    double radBearing = Math.toRadians(desiredTag.ftcPose.bearing); //JAVA ONLY WORKS IN RADIANS
                    double strafe = desiredTag.ftcPose.range * Math.cos(radBearing); // AB = AC * cos(<BAC)
                    if (desiredTag.ftcPose.bearing < 0) { //IF TAG IS TO THE LEFT, INVERSE STRAFE
                        strafe *= -1;
                    }
                    double forward = sqrt(Math.pow(desiredTag.ftcPose.range, 2) - Math.pow(strafe, 2)); //BC = sqrt(AB^2 - AC^2)
                    telemetry.addData(">", "ATTEMPT 2");
                    telemetry.addData("FWD = %2f", forward);
                    telemetry.addData("STRAFE = %2f", strafe);
                    robot.drive(forward * inch, strafe * inch, .5);
                    robot.drive(0, 7 * inch, .5); // STRAFE B/C CAMERA IS ON LEFT AND DROPPER IS RIGHT (*COULD BE -7, I DONT REMEMBER*)
                }


                //.............................*ATTEMPT INCREMENTIVLY*............................. (I LIKE THIS ONE)
                if (attempt == 3) { //***************B/C OF INCREMENT, WE NEED UPDATES OF THE AT SO ABSALUTLY, POSITUTLY, NO WHILE LOOPS
                    String side = "";
                    if (desiredTag.ftcPose.bearing < 0) {
                        side = "moveL";
                    } else {
                        side = "moveR";
                    }
                    telemetry.addData("side = %s", side);

                    if (side == "moveL") {
                        if (desiredTag.ftcPose.bearing > -3) { //GIVE WHILE LOOP A BUFFER OF 3 DEG MEANING while (!within 3 deg)
                            robot.drive(0, -1 * inch, .5); //MOVE L 1in
                            telemetry.addData("Current Bearing = %2f", desiredTag.ftcPose.bearing);
                        }
                    } else if (side == "moveR") {
                        if (desiredTag.ftcPose.bearing < 3) { //GIVE WHILE LOOP A BUFFER OF 3 DEG MEANING while (!within 3 deg)
                            robot.drive(0, 1 * inch, .5); //MOVE R 1in
                            telemetry.addData("Current Bearing = %2f", desiredTag.ftcPose.bearing);
                        }
                    }

                    if (desiredTag.ftcPose.range > 7) { //GET WITHIN 7in
                        robot.drive(1 * inch, 0, .5); //MOVE 1in FWD
                        telemetry.addData("Current Range = %2f", desiredTag.ftcPose.range);
                    }
                    robot.drive(0, 7 * inch, .5); // STRAFE B/C CAMERA IS ON LEFT AND DROPPER IS RIGHT (*COULD BE -7, I DONT REMEMBER*)
                }
            }else {
                //-----------------------------MANUAL----------------------------- (WORKS)
                motorFL.setPower(((this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) + ((this.gamepad1.left_stick_y)) - (this.gamepad1.left_stick_x)) * 1);
                motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) - (this.gamepad1.left_stick_x)) * 1);
                motorBR.setPower((-(this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * 1);
                motorFR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * 1);
                telemetry.addData(">", "Manual");
            }
            telemetry.update();
        }
    }

    //*****************************FUNCS***************************** (WORKS)
    //-----------------------------INIT AT-----------------------------
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

    //-----------------------------HARDWARE INIT-----------------------------
    void hardwareSetup(){
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    //-----------------------------CAM EXPOSURE-----------------------------
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
        telemetry.update();
    }

        public void moveRobot(double x, double y, double yaw) { //TAKEN FROM EXAMPLE. NOT SURE HOW IT WORKS. ONLY USED IN EXAMPLE 1
            // Calculate wheel powers.
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
            motorFL.setPower(leftFrontPower);
            motorFR.setPower(rightFrontPower);
            motorBL.setPower(leftBackPower);
            motorBR.setPower(rightBackPower);
            telemetry.addData("motorFL pwr = %3f", leftFrontPower);
            telemetry.addData("motorFR pwr = %3f", rightFrontPower);
            telemetry.addData("motorBL pwr = %3f", leftBackPower);
            telemetry.addData("motorBR pwr = %3f", rightBackPower);
        }

    }


