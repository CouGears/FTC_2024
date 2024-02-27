package org.firstinspires.ftc.teamcode;

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
public class OneDiver_CompetitionDriving2024 extends LinearOpMode {
    private DigitalChannel red1,red2, red3, green1, green2, green3;
    private ColorSensor color, color2;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift, PullUp;
    public static CRServo IntakeString;
    public static Servo DropServo, AirplaneLaunch;

    public static DistanceSensor BackdropDistance;
    private AutonMethods robot = new AutonMethods();
    public int driveswitch = 1;
    private int pixel_color = 0;

    public int intakemode = 0;
    private int liftLimit = 3000;
    private boolean pullup = false;
    private int dropDelay = 0;
    public ElapsedTime mRunTime = new ElapsedTime();

    public void TelemetryUpdate() {
        telemetry.addData("Drive Mode", driveswitch);
        telemetry.addLine();
        telemetry.addData("Intake Mode", intakemode);
        telemetry.addLine();
        telemetry.addData("Servo Position", IntakeString.getPower());
        telemetry.addLine();
        telemetry.addData("Lift Pos = ", Lift.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Drop Servo Pos = ", DropServo.getPosition());
        telemetry.addLine();
        telemetry.addData("Airplane Launch Pos = ", AirplaneLaunch.getPosition());
        telemetry.addLine();
        telemetry.addData("PullUp Pos = ", PullUp.getCurrentPosition());
        telemetry.update();
    }
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
        PullUp.setDirection(DcMotorSimple.Direction.FORWARD);

        Lift = hardwareMap.get(DcMotor.class, "Lift");
        //Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        mRunTime.reset();
        while (opModeIsActive()) {
            TelemetryUpdate();

            while (!pullup) {
                double speed = 1;
                if (driveswitch == 0) {
                    speed = 1;
                } else if (driveswitch == 1) {
                    speed = .66;
                }
                if (gamepad1.dpad_left) {
                    dropDelay++;
                    if (dropDelay >= 10){
                        DropServo.setPosition(.5);
                        dropDelay = 0;
                    }
                } else {
                    DropServo.setPosition(.045);
                }
                if (gamepad1.start) {
                    Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                if (gamepad1.x) { // P1 can control this, not too hard
                    AirplaneLaunch.setPosition(0);
                } else {
                    AirplaneLaunch.setPosition(1);
                }


                if (gamepad1.a && driveswitch == 1) {
                    driveswitch = 0;
                } else if (gamepad1.b && driveswitch == 0) {
                    driveswitch = 1;
                }
                if (gamepad1.y || (gamepad1.right_bumper && gamepad1.left_bumper)) {
                    BackIntake.setPower(1);
                    MiddleIntake.setPower(-1);
                } else if (gamepad1.right_bumper) {
                    BackIntake.setPower(1);
                    MiddleIntake.setPower(0);
                } else if (gamepad1.left_bumper) {
                    MiddleIntake.setPower(-1);
                    BackIntake.setPower(0);
                } else {
                    MiddleIntake.setPower(0);
                    BackIntake.setPower(0);
                }
                if (gamepad1.left_stick_button) {
                    // Control to pull up
                    PullUp.setPower(1.0);
                } else if (gamepad1.right_stick_button) {
                    // Control to release down
                    pullup = true;
                } else {
                    // Ensure the motor stops if no buttons are pressed and not in active mode
                    PullUp.setPower(0);
                }


                motorFL.setPower(((this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) + ((this.gamepad1.left_stick_y)) - (this.gamepad1.left_stick_x)) * speed);
                motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) - (this.gamepad1.left_stick_x)) * speed);
                motorBR.setPower((-(this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * speed);
                motorFR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * speed);


                if ((gamepad1.right_trigger > .1) || (5 < mRunTime.time() && mRunTime.time() < 10))
                {
                    IntakeString.setPower(gamepad1.right_trigger);
                } else if (gamepad1.left_trigger > .1) {
                    IntakeString.setPower(-1 * gamepad1.left_trigger);
                } else {
                    IntakeString.setPower(0.0);
                }
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
                        telemetry.addData("Middle intake:", "Green Pixel Loaded");
                        break;
                    case 2:
                        //telemetry.addLine("Reading: purple");
                        telemetry.addData("Middle intake:", "Purple Pixel Loaded");

                        break;
                    case 3:
                        //telemetry.addLine("Reading: yellow");
                        telemetry.addData("Middle intake:", "Yellow Pixel Loaded");

                        break;
                    case 4:
                        //telemetry.addLine("Reading: white");
                        telemetry.addData("Middle intake:", "White Pixel Loaded");

                        break;
                    case 0:
                        telemetry.addData("Middle intake:", "No Pixel Sensed");
                        // telemetry.addLine("Reading: No Pixel");
                        break;

                }



                telemetry.update();

                double distanceFromBackdrop = BackdropDistance.getDistance(DistanceUnit.INCH);

                double radian120 = 120 * (3.14/180);
                double radian15 = 15 * (3.14/180);

                double sin120 = Math.sin(radian120);
                double sin15 = Math.sin(radian15);

                double liftLimitInches = distanceFromBackdrop * (sin120 / sin15);



                //LIFT
                if ((gamepad1.dpad_up && Lift.getCurrentPosition() <= liftLimit) || (gamepad1.dpad_up && gamepad1.dpad_right)) { // P1 should still be in control of lift
                    Lift.setPower(1);
                } else if ((gamepad1.dpad_down && Lift.getCurrentPosition() >= 0) || (gamepad1.dpad_down && gamepad1.dpad_right)) { //At 500 b/c motor will overspin w/ momentum and end up <0
                    Lift.setPower(-1);
                } else {
                    Lift.setPower(0);
                }


            }
            while (pullup) {
                if (gamepad1.y) {
                    pullup = false;
                }
                PullUp.setPower(-1.0);
            }
        }
    }
}





