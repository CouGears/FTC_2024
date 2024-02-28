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
    // DIGITAL CHANNELS
    private DigitalChannel red1,red2, red3, green1, green2, green3;
    // COLOR SENSORS
    private ColorSensor color, color2;
    // DC MOTORS
    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift, PullUp;
    // CONTINUOUS ROTATION SERVO
    public static CRServo IntakeString;
    // SERVOS
    public static Servo DropServo, AirplaneLaunch;
    // DISTANCE SENSOR (FRONT)
    public static DistanceSensor BackdropDistance;
    // VARIABLE FOR COLOR SENSOR SYSTEM
    private int pixel_color = 0;
    // LIFT LIMIT
    private int liftLimit = 600;
    // VARIABLE FOR PULLUP SYSTEMS A & B
    private boolean pullup = false;
    // NOT SURE WHAT THIS DOES
    private int dropDelay = 0;
    // RUN TIME VARIABLE
    public ElapsedTime mRunTime = new ElapsedTime();
    // SPEED OF THE ROBOT
    // RANGE: [-1, 1]
    // 1 = FULL SPEED
    // -1 = REVERSE FULL SPEED
    private double speed = 1;
    // VARIABLE TO KEEP TRACK OF THE MODE/SPEED OF THE ROBOT
    private String mode = "FAST";
    // VARIABLE TO KEEP TRACK OF THE STATE OF THE BUCKET SYSTEM
    private String bucketstate = "INITIAL POSITION";
    // VARIABLE TO KEEP TRACK OF THE STATE OF THE DRONE SYSTEM
    private String dronestate = "ARMED";
    // VARIABLE TO KEEP TRACK OF THE STATE OF THE PULLUP SYSTEM
    private String pullupstate = "STATIONARY";

    // TELEMETRY UPDATING FUNCIOTN
    public void TelemetryUpdate() {
        // ADD DATA
        telemetry.addData("Speed", mode);
        telemetry.addData("Bucket Position", bucketstate);
        telemetry.addData("Lift Position", Lift.getCurrentPosition());
        telemetry.addData("Drone State", dronestate);
        telemetry.addData("Pullup Position", PullUp.getCurrentPosition());
        telemetry.addData("Pullup State", pullupstate);
        // UPDATE TELEMETRY ON DRIVER STATION
        telemetry.update();
    }
    // RUN ON ROBOT INIT
    @Override
    public void runOpMode() {
        //// MAP HARDWARE TO OBJECTS
        // MAP DC MOTORS
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        BackIntake = hardwareMap.get(DcMotor.class, "BackIntake");
        MiddleIntake = hardwareMap.get(DcMotor.class, "MiddleIntake");
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        // MAP CONTINUOUS ROTATION SERVO
        IntakeString = hardwareMap.get(CRServo.class, "IntakeString");
        // MAP SERVOS
        DropServo = hardwareMap.get(Servo.class, "DropServo");
        AirplaneLaunch = hardwareMap.get(Servo.class, "AirplaneLaunch");
        // MAP DC MOTOR
        PullUp = hardwareMap.get(DcMotor.class, "PullUp");

        // MAP DIGITAL CHANNELS
        red1 = hardwareMap.get(DigitalChannel.class, "red1");
        green1 = hardwareMap.get(DigitalChannel.class, "green1");
        red2 = hardwareMap.get(DigitalChannel.class, "red2");
        green2 = hardwareMap.get(DigitalChannel.class, "green2");
        red3 = hardwareMap.get(DigitalChannel.class, "red3");
        green3 = hardwareMap.get(DigitalChannel.class, "green3");
        color = hardwareMap.get(ColorSensor.class, "Color");
        color2 = hardwareMap.get(ColorSensor.class, "Color2");

        // MAP DISTANCE SENSOR
        BackdropDistance = hardwareMap.get(DistanceSensor.class, "BackdropDistance");


        // SET DC MOTOR ZERO POWER BEHAVIORS
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MiddleIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PullUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // SET DC MOTOR DIRECTIONS
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        MiddleIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        PullUp.setDirection(DcMotorSimple.Direction.FORWARD);

        // ADD DATA TO TELEMETRY
        telemetry.addData("Status", "Initialized");
        // UPDATE TELEMETRY ON DRIVER STATION
        telemetry.update();

        // WAIT FOR OPMODE START
        waitForStart();
        // RESET RUNTIME (ON START)
        mRunTime.reset();
        // UPDATE TELEMETRY
        TelemetryUpdate();
        // WHILE ROBOT IS NOT ACTIVELY PULLING UP & OPMODE IS ACTIVE
        while (!pullup && opModeIsActive()) {
            //// CALL SYSTEM FUNCTIONS
            // CALL BUCKET SYSTEM FUNCTION
            bucketSystem();
            // CALL DRONE SYSTEM FUNCTION
            droneSystem();
            // CALL SPEED SYSTEM FUNCTION
            speedSystem();
            // CALL INTAKE SYSTEM FUNCTION
            intakeSystem();
            // CALL PULLUP SYSTEM A FUNCTION
            pullupSystemA();;
            // CALL DRIVE SYSTEM FUNCTION MODE 1
            // MODE 0: REGULAR DRIVE SYSTEM
            // MODE 1: DRIVE SYSTEM THAT WILL NOT LET YOU DRIVE FORWARDS IF THE BUCKET IS
            // (cont.) TOUCHING THE BACKDROP
            driveSystem(1);
            // CALL INTAKE STRING/PULLEY SYSTEM
            intakeStringSystem();
            // CALL MIDDLE INTAKE PIXEL COLOR SENSOR SYSTEM
            middleIntakePixelColorSystem();
            // CALL LIFT SYSTEM
            liftSystem();
            // UPDATE TELEMETRY WITH UPDATED DATA
            TelemetryUpdate();
        }
        // WHILE ROBOT IS ACTIVELY PULLING UP
        while (pullup && opModeIsActive()) {
            // CALL PULLUP SYSTEM B
            pullupSystemB();
        }

    }

    //// DRIVE SYSTEM
    /// KEYBINDS - CONTROLLER 1
    // LEFT JOYSTICK: DRIVE/STRAFE
    // RIGHT JOYSTICK: DRIVE/TURN
    private void driveSystem(int mode) {
        // RUN DIFFERENT DRIVE SYSTEM DEPENDING ON MODE
        switch (mode) {
            // IF MODE IS 0
            case 0:
                // STANDARD DRIVE SYSTEM
                motorFL.setPower(((gamepad1.right_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                motorBL.setPower(-(-(gamepad1.right_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                break;
            // IF MODE IS 1
            case 1:
                // IF THE LIFT IS AT IT'S LIMIT & BOTH JOYSTICKS HAVE POSITIVE X VALUES
                if (Lift.getCurrentPosition() == liftLimit && gamepad1.right_stick_x > 0 && gamepad1.left_stick_x > 0) {
                    // IGNORE THE X VALUES OF BOTH JOYSTICKS
                    motorFL.setPower(((gamepad1.right_stick_y) + (gamepad1.left_stick_y)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) - (gamepad1.left_stick_y)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.left_stick_y)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.left_stick_y)) * speed);
                }
                // IF THE LIFT IS AT IT'S LIMIT & THE RIGHT JOYSTICK HAS A POSITIVE X VALUE
                else if (Lift.getCurrentPosition() == liftLimit && gamepad1.right_stick_x > 0) {
                    // IGNORE THE X VALUES OF THE RIGHT JOYSTICK
                    motorFL.setPower(((gamepad1.right_stick_y) + ((gamepad1.left_stick_y)) - (gamepad1.left_stick_x)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) - (gamepad1.left_stick_y) - (gamepad1.left_stick_x)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.left_stick_y) + (gamepad1.left_stick_x)) * speed);
                }
                // IF THE LIFT IS AT IT'S LIMIT & THE LEFT JOYSTICK HAS A POSITIVE X VALUE
                else if (Lift.getCurrentPosition() == liftLimit && gamepad1.left_stick_x > 0) {
                    // IGNORE THE X VALUES OF THE LEFT JOYSTICK
                    motorFL.setPower(((gamepad1.right_stick_y) - (gamepad1.right_stick_x) + (gamepad1.left_stick_y)) * speed);
                    motorBL.setPower(-(-(gamepad1.right_stick_y) + (gamepad1.right_stick_x) - (gamepad1.left_stick_y)) * speed);
                    motorBR.setPower((-(gamepad1.right_stick_y) - (gamepad1.right_stick_x) - (gamepad1.left_stick_y)) * speed);
                    motorFR.setPower(-((gamepad1.right_stick_y) + (gamepad1.right_stick_x) + (gamepad1.left_stick_y)) * speed);
                }
                // IF THE LIFT IS NOT AT IT'S LIMIT OR BOTH OF THE JOYSTICKS DO NOT HAVE POSITIVE VALUES
                else {
                    // STANDARD DRIVE SYSTEM
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
        // IF CONTROLLER 1'S Y BUTTON IS PRESSED OR BOTH OF CONTROLLER 1'S BUMPERS ARE PRESSED
        if (gamepad1.y || (gamepad1.right_bumper && gamepad1.left_bumper)) {
            // RUN BOTH INTAKES
            BackIntake.setPower(1);
            MiddleIntake.setPower(-1);
        }
        // IF CONTROLLER 1'S X BUTTON IS PRESSED OR BOTH OF CONTROLLER 1'S TRIGGERS ARE PRESSED
        else if (gamepad1.x || ((gamepad1.right_trigger > 0) && (gamepad1.left_trigger > 0))) {
            // RUN BOTH INTAKES IN REVERSE
            BackIntake.setPower(-1);
            MiddleIntake.setPower(1);
        }
        // IF CONTROLLER 1'S RIGHT BUMPER IS PRESSED
        else if (gamepad1.right_bumper) {
            // RUN BACK INTAKE ONLY
            BackIntake.setPower(1);
            MiddleIntake.setPower(0);
        }
        // IF CONTROLLER 1'S LEFT BUMPER IS PRESSED
        else if (gamepad1.left_bumper) {
            // RUN MIDDLE INTAKE ONLY
            MiddleIntake.setPower(-1);
            BackIntake.setPower(0);
        }
        // IF CONTROLLER 1'S RIGHT TRIGGER IS PRESSED
        else if (gamepad1.right_trigger > 0) {
            // RUN BACK INTAKE ONLY IN REVERSE
            BackIntake.setPower(-1);
            MiddleIntake.setPower(0);
        }
        // IF CONTROLLER 1'S LEFT TRIGGER IS PRESSED
        else if (gamepad1.left_trigger > 0) {
            // RUN MIDDLE INTAKE ONLY IN REVERSE
            MiddleIntake.setPower(1);
            BackIntake.setPower(0);
        }
        // IF CONTROLLER 1'S X BUTTON, Y BUTTON, LEFT TRIGGER, RIGHT TRIGGER, LEFT BUMPER, AND RIGHT
        // BUMPER ARE ALL NOT PRESSED
        else {
            // DON'T RUN ANY INTAKES
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
        // INITIALIZE AND DEFINE VARIABLE FOR ROBOT'S DISTANCE FROM BACKDROP IN INCHES
        double distanceFromBackdrop = BackdropDistance.getDistance(DistanceUnit.INCH);

        // INITIALIZE AND DEFINE VARIABLE FOR 120 DEGREES IN RADIANS
        final double radian120 = 120 * (3.14/180);
        // INITIALIZE AND DEFINE VARIABLE FOR 15 DEGREES IN RADIANS
        final double radian15 = 15 * (3.14/180);

        // INITIALIZE AND DEFINE VARIABLE FOR THE SINE OF 120 DEGREES
        final double sin120 = Math.sin(radian120);
        // INITIALIZE AND DEFINE VARIABLE FOR THE SINE OF 15 DEGREES
        final double sin15 = Math.sin(radian15);
        /// INITIALIZE AND DEFINE VARIABLE FOR THE LIFT LIMIT IN INCHES USING:
        // THE ROBOT'S DISTANCE FROM THE BACKDROP IN INCHES
        // THE SINE OF 120 DEGREES
        // THE SINE OF 15 DEGREES
        // AN ADJUSTMENT CONSTANT
        double liftLimitInches = distanceFromBackdrop * (sin120 / sin15) - 4.72;
        // INITIALIZE AND DEFINE THE AMOUNT OF MOTOR ENCODER UNITS (1620 RPM DC MOTOR) FOR 1 INCH OF LINEAR SLIDE MOVEMENT
        final int liftEncoderPerInch = 22;
        // INITIALIZE AND DEFINE VARIABLE FOR THE LIFT LIMIT IN MOTOR ENCODER UNITS (1620 RPM DC MOTOR)
        int liftLimit = (int) (liftEncoderPerInch * liftLimitInches);
        // IF THE CALCULATED LIFT LIMIT IS MORE THAN THE ULTIMATE MAXIMUM LIFT LIMIT (FULLY EXTENDED LINEAR SLIDE)
        if (liftLimit > 600) {
            // SET THE LIFT LIMIT TO THE ULTIMATE MAXIMUM LIFT LIMIT (FULLY EXTENDED LINEAR SLIDE)
            liftLimit = 600;
        }

        // IF CONTROLLER 1'S D-PAD UP BUTTON IS PRESSED AND THE LIFT IS NOT AT IT'S LIMIT
        // OR CONTROLLER 1'S D-PAD UP BUTTON IS PRESSED AND CONTROLLER 1'S D-PAD RIGHT BUTTON IS PRESSED (OVERRIDE LIMIT BUTTON)
        if ((gamepad1.dpad_up && Lift.getCurrentPosition() <= liftLimit) || (gamepad1.dpad_up && gamepad1.dpad_right)) {
            // RAISE THE LIFT AT FULL SPEED
            Lift.setPower(1);
        }
        // IF CONTROLLER 1'S D-PAD DOWN BUTTON IS PRESSED AND THE LIFT IS NOT AT IT'S LOWEST POSSIBLE POSITION
        // OR CONTROLLER 1'S D-PAD DOWN BUTTON IS PRESSED AND CONTROLLER 1'S D-PAD RIGHT BUTTON IS PRESSED (OVERRIDE LIMIT BUTTON)
        else if ((gamepad1.dpad_down && Lift.getCurrentPosition() >= 0) || (gamepad1.dpad_down && gamepad1.dpad_right)) {
            // LOWER THE LIFT AT HALF SPEED
            Lift.setPower(-.5);
        }
        // IF CONTROLLER 1'S D-PAD UP BUTTON IS NOT PRESSED OR THE LIFT IS AT IT'S LIMIT AND CONTROLLER 1'S D-PAD RIGHT BUTTON IS NOT PRESSED
        // AND CONTROLLER 1'S D-PAD DOWN BUTTON IS NOT PRESSED OR THE LIFT IS AT IT'S LOWEST POSITION AND CONTROLLER 1'S D-PAD RIGHT BUTTON IS NOT PRESSED
        else {
            // DON'T MOVE THE LIFT
            Lift.setPower(0);
        }
    }

    //// PULLUP SYSTEM FOR WHEN ROBOT IS NOT ACTIVELY PULLING UP
    /// KEYBINDS - CONTROLLER 2
    // D-PAD UP: EXTEND PULLUP ARM
    // D-PAD DOWN: RETRACT PULLUP ARM (PULLUP) AND SHUT DOWN DRIVER CONTROLS
    private void pullupSystemA() {
        // IF CONTROLLER 2'S D-PAD UP BUTTON IS PRESSED
        if (gamepad2.dpad_up) {
            // RAISE THE PULLUP HOOK
            PullUp.setPower(1.0);
            // UPDATE THE PULLUP STATE VARIABLE FOR TELEMETRY
            pullupstate = "EXTENDING";
        }
        // IF CONTROLLER 2'S D-PAD DOWN BUTTON IS PRESSED
        else if (gamepad1.dpad_down) {
            // CALL PULLUP SYSTEM B AFTER THE NEXT TELEMETRY UPDATE AND PAUSE ALL OTHER SYSTEMS FROM BEING CALLED
            pullup = true;
            // UPDATE THE PULLUP STATE VARIABLE FOR TELEMETRY
            pullupstate = "RETRACTING";
        }
        // IF CONTROLLER 2'S D-PAD UP BUTTON IS NOT PRESSED AND CONTROLLER 2'S D-PAD DOWN BUTTON IS NOT PRESSED
        else {
            // DO NOT MOVE THE PULLUP HOOK
            PullUp.setPower(0);
            // UPDATE THE PULLUP STATE VARIABLE FOR TELEMETRY
            pullupstate = "STATIONARY";
        }
    }

    //// PULLUP SYSTEM FOR WHEN ROBOT IS ACTIVELY PULLING UP
    /// KEYBINDS - CONTROLLER 2
    // D-PAD LEFT: STOP PULLING UP AND RETURN TO DRIVER CONTROL
    // D-PAD RIGHT: STOP PULLING UP AND RETURN TO DRIVER CONTROL
    private void pullupSystemB() {
        // IF CONTROLLER 2'S D-PAD LEFT BUTTON IS PRESSED OR CONTROLLER 2'S D-PAD RIGHT BUTTON IS PRESSED
        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            // CALL OTHER SYSTEMS AFTER THE NEXT TELEMETRY UPDATE
            pullup = false;
        }
        // IF THE ROBOT IS HOOKED ON TO THE PULLUP BAR: PULL THE ROBOT UP
        // IF THE ROBOT IS NOT HOOKED ON TO THE PULLUP BAR: LOWER THE PULLUP HOOK
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





