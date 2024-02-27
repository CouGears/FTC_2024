package org.firstinspires.ftc.teamcode.JoshTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class TwoDriverTeleOp extends LinearOpMode {
    //// DEVICE DEFINITIONS
    // JACOB COLOR SENSOR
    private DigitalChannel red1,red2, red3, green1, green2, green3;
    private ColorSensor color, color2;

    // DC MOTORS
    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift, PullUp;
    // CONTINUOUS SERVO
    public static CRServo IntakeString;
    // SERVOS
    public static Servo DropServo, AirplaneLaunch;
    // DISTANCE SENSOR (FRONT)
    public static DistanceSensor BackdropDistance;
    // JACOB COLOR SENSOR
    private int pixel_color = 0;
    // lift limit
    private int liftLimit = 600;
    // pulling up variable i guess
    private boolean pullup = false;
    // not sure what drop delay does
    private int dropDelay = 0;
    // run time variable
    public ElapsedTime mRunTime = new ElapsedTime();
    // robot speed variable
    private double speed = 1;

    private String mode = "FAST";

    private String bucketstate = "INITIAL POSITION";

    private String dronestate = "ARMED";

    private String pullupstate = "STATIONARY";

    // function to update telemetry
    public void TelemetryUpdate() {
        telemetry.addData("Speed", mode);
        telemetry.addData("Bucket Positio", bucketstate);
        telemetry.addData("Lift Positio", Lift.getCurrentPosition());
        telemetry.addData("Drone Stat", dronestate);
        telemetry.addData("Pullup Pos", PullUp.getCurrentPosition());
        telemetry.addData("Pullup State", pullupstate);
        telemetry.update();
    }
    // init function
    @Override
    public void runOpMode() {
        //region hardware map
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        BackIntake = hardwareMap.get(DcMotor.class, "BackIntake");
        MiddleIntake = hardwareMap.get(DcMotor.class, "MiddleIntake");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        IntakeString = hardwareMap.get(CRServo.class, "IntakeString");
        DropServo = hardwareMap.get(Servo.class, "DropServo");
        AirplaneLaunch = hardwareMap.get(Servo.class, "AirplaneLaunch");
        PullUp = hardwareMap.get(DcMotor.class, "PullUp");

        red1= hardwareMap.get(DigitalChannel.class, "red1");
        green1= hardwareMap.get(DigitalChannel.class, "green1");
        red2= hardwareMap.get(DigitalChannel.class, "red2");
        green2= hardwareMap.get(DigitalChannel.class, "green2");
        red3= hardwareMap.get(DigitalChannel.class, "red3");
        green3= hardwareMap.get(DigitalChannel.class, "green3");
        color = hardwareMap.get(ColorSensor.class, "Color");
        color2 =hardwareMap.get(ColorSensor.class, "Color2");

        BackdropDistance = hardwareMap.get(DistanceSensor.class, "BackdropDistance");


        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MiddleIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PullUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        MiddleIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        PullUp.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        mRunTime.reset();
        // on teleop start
        TelemetryUpdate();
        while (!pullup && opModeIsActive()) {
            bucketSystem();
            droneSystem();
            speedSystem();
            intakeSystem();
            pullupSystemA();;
            driveSystem(0);
            intakeStringSystem();
            middleIntakePixelColorSystem();
            liftSystem();
            TelemetryUpdate();
        }
        while (pullup && opModeIsActive()) {
            pullupSystemB();
        }

    }

    //// DRIVE SYSTEM
    /// KEYBINDS - CONTROLLER 1
    // LEFT JOYSTICK: DRIVE/STRAFE
    // RIGHT JOYSTICK: DRIVE/TURN
    private void driveSystem(int mode) {
        switch (mode) {
            case 0:
                motorFL.setPower(((gamepad1.right_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                motorBL.setPower(-(-(gamepad1.right_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                break;
            case 1:
                if (Lift.getCurrentPosition() == liftLimit && gamepad1.right_stick_x > 0 && gamepad1.left_stick_x > 0) {
                    motorFL.setPower(((gamepad1.right_stick_y) + (gamepad1.left_stick_y)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) - (gamepad1.left_stick_y)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.left_stick_y)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.left_stick_y)) * speed);
                } else if (Lift.getCurrentPosition() == liftLimit && gamepad1.right_stick_x > 0) {
                    motorFL.setPower(((gamepad1.right_stick_y) + ((gamepad1.left_stick_y)) - (gamepad1.left_stick_x)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) - (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                } else if (Lift.getCurrentPosition() == liftLimit && gamepad1.left_stick_x > 0) {
                    motorFL.setPower(((gamepad1.right_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_y)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_y)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_y)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_y)) * speed);
                } else {
                    motorFL.setPower(((gamepad1.right_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                }
        }
    }
    //// INTAKE SYSTEM
    /// KEYBINDS - CONTROLLER 1
    // Y: BOTH INTAKES
    // X: REVERSE BOTH INTAKES
    // RIGHT BUMPER: BACK INTAKE
    // LEFT BUMPER: MIDDLE INTAKE
    // RIGHT TRIGGER: REVERSE BACK INTAKE
    // LEFT TRIGGER: REVERSE MIDDLE INTAKE
    private void intakeSystem() {
        if (gamepad1.y || (gamepad1.right_bumper && gamepad1.left_bumper)) {
            BackIntake.setPower(1);
            MiddleIntake.setPower(-1);
        } else if (gamepad1.x || ((gamepad1.right_trigger > 0) && (gamepad1.left_trigger > 0))) {
            BackIntake.setPower(-1);
            MiddleIntake.setPower(1);
        }  else if (gamepad1.right_bumper) {
            BackIntake.setPower(1);
            MiddleIntake.setPower(0);
        } else if (gamepad1.left_bumper) {
            MiddleIntake.setPower(-1);
            BackIntake.setPower(0);
        } else if (gamepad1.right_trigger > 0) {
            BackIntake.setPower(-1);
            MiddleIntake.setPower(0);
        } else if (gamepad1.left_trigger > 0) {
            MiddleIntake.setPower(1);
            BackIntake.setPower(0);
        } else {
            MiddleIntake.setPower(0);
            BackIntake.setPower(0);
        }
    }

    //// LIFT SYSTEM
    /// KEYBINDS - CONTROLLER 1
    // D-PAD UP: RAISE LIFT
    // D-PAD DOWN: LOWER LIFT
    // D-PAD RIGHT: OVERRIDE LIFT LIMIT
    private void liftSystem() {
        // robot distance from backdrop
        double distanceFromBackdrop = BackdropDistance.getDistance(DistanceUnit.INCH);

        // 120 degrees in radians
        double radian120 = 120 * (3.14/180);
        // 15 degrees in radians
        double radian15 = 15 * (3.14/180);

        // sine of 120 degrees
        double sin120 = Math.sin(radian120);
        // sine of 15 degrees
        double sin15 = Math.sin(radian15);
        // lift limit in inches
        double liftLimitInches = distanceFromBackdrop * (sin120 / sin15) - 4.72;
        // lift motor encoder units per inch of linear slide movement
        int liftEncoderPerInch = 22;
        // lift limit in motor encoder units
        int liftLimit = (int) (liftEncoderPerInch * liftLimitInches);
        // if the limit is calculated to be more than the ultimate maximum (fully extended linear slide)
        if (liftLimit > 600) {
            // set the limit to the ultimate maximum (fully extended linear slide)
            liftLimit = 600;
        }

        if ((gamepad1.dpad_up && Lift.getCurrentPosition() <= liftLimit) || (gamepad1.dpad_up && gamepad1.dpad_right)) {
            Lift.setPower(1);
        } else if ((gamepad1.dpad_down && Lift.getCurrentPosition() >= 0) || (gamepad1.dpad_down && gamepad1.dpad_right)) {
            Lift.setPower(-.5);
        } else {
            Lift.setPower(0);
        }
    }

    //// PULLUP SYSTEM FOR WHEN ROBOT IS NOT ACTIVELY PULLING UP
    /// KEYBINDS - CONTROLLER 2
    // D-PAD UP: EXTEND PULLUP ARM
    // D-PAD DOWN: RETRACT PULLUP ARM (PULLUP) AND SHUT DOWN DRIVER CONTROLS
    private void pullupSystemA() {
        if (gamepad2.dpad_up) {
            // Control to pull up
            PullUp.setPower(1.0);
            pullupstate = "EXTENDING";
        } else if (gamepad1.dpad_down) {
            // Control to release down
            pullup = true;
            pullupstate = "RETRACTING";
        } else {
            // Ensure the motor stops if no buttons are pressed and not in active mode
            PullUp.setPower(0);
            pullupstate = "STATIONARY";
        }
    }

    //// PULLUP SYSTEM FOR WHEN ROBOT IS ACTIVELY PULLING UP
    /// KEYBINDS - CONTROLLER 2
    // D-PAD LEFT: STOP PULLING UP AND RETURN TO DRIVER CONTROL
    // D-PAD RIGHT: STOP PULLING UP AND RETURN TO DRIVER CONTROL
    private void pullupSystemB() {
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            pullup = false;
        }
        PullUp.setPower(-1.0);
    }

    //// INTAKE STRING/PULLEY SYSTEM
    /// KEYBINDS - CONTROLLER 2
    // RIGHT TRIGGER: LOWER BACK INTAKE SYSTEM
    // LEFT TRIGGER: RAISE BACK INTAKE SYSTEM
    private void intakeStringSystem() {
        if ((gamepad2.right_trigger > .1) || (5 < mRunTime.time() && mRunTime.time() < 10))
        {
            IntakeString.setPower(gamepad2.right_trigger);
        } else if (gamepad2.left_trigger > .1) {
            IntakeString.setPower(-gamepad2.left_trigger);
        } else {
            IntakeString.setPower(0.0);
        }
    }

    //// ROBOT MODE (SPEED) SYSTEM CONTROLLER
    /// KEYBINDS - CONTROLLER 1
    // A: FAST MODE
    // B: SLOW MODE
    private void speedSystem() {
        if (gamepad1.a) {
            speed = 1;
            mode = "FAST";

        } else if (gamepad1.b) {
            speed = 0.5;
            mode = "SLOW";
        }
    }


    //// DRONE (AIRPLANE) SYSTEM
    /// KEYBINDS - CONTROLLER 2
    // A: LAUNCH DRONE
    private void droneSystem() {
        if (gamepad2.a) {
            AirplaneLaunch.setPosition(0);
        } else {
            AirplaneLaunch.setPosition(1);
            dronestate = "LAUNCHED";
        }
    }

    //// BUCKET (DROP SERVO) SYSTEM
    /// KEYBINDS - CONTROLLER 1
    // D-PAD LEFT: DROP PIXEL (FROM BUCKET)
    private void bucketSystem() {
        if (gamepad1.dpad_left) {
            dropDelay++;
            if (dropDelay >= 10){
                DropServo.setPosition(.5);
                dropDelay = 0;
                bucketstate = "DROPPING";
            }
        } else {
            DropServo.setPosition(.045);
            bucketstate = "INITIAL POSITION";
        }
    }

    private void middleIntakePixelColorSystem() {
        // color sensors 1- green 2-purple 3- yellow all- white (4)
        if ((color.green()> 2000 && color.blue()> 2000)||(color2.green()> 2000 && color2.blue()> 2000)) {
            pixel_color = 4;
        } else if ((color.green()> 1000&& color.red()> 850 && color.blue() < 1000) || (color2.green()> 1000&& color2.red()> 850 && color2.blue() < 1000)) {
            pixel_color = 3;
        } else if ((color.green()> 1000 && color.blue()< 1000)||(color2.green()> 1000 && color2.blue()< 1000) ) {
            pixel_color = 1;
        } else if ((color.blue() > 1550 && color.red() < 1380) || (color2.blue() > 1550 && color2.red() < 1380)) {
            pixel_color = 2;
        }

        else{ pixel_color = 0;}
        switch (pixel_color){
            case 1:
                // telemetry.addLine("Reading: green");
                telemetry.addData("Middle intake", "Green Pixel Loaded");
                break;
            case 2:
                //telemetry.addLine("Reading: purple");
                telemetry.addData("Middle intake", "Purple Pixel Loaded");

                break;
            case 3:
                //telemetry.addLine("Reading: yellow");
                telemetry.addData("Middle intake", "Yellow Pixel Loaded");

                break;
            case 4:
                //telemetry.addLine("Reading: white");
                telemetry.addData("Middle intake", "White Pixel Loaded");

                break;
            case 0:
                telemetry.addData("Middle intake", "No Pixel Sensed");
                // telemetry.addLine("Reading: No Pixel");
                break;

        }
    }

}





