package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class TwoDrivers_CompetitionDriving2024 extends LinearOpMode {

    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift, PullUp;
    public static CRServo IntakeString;
    public static Servo DropServo, AirplaneLaunch;
    private AutonMethods robot = new AutonMethods();
    public int driveswitch = 1;

    public int intakemode = 0;
    private int liftLimit = 3000;
    private boolean pullup = false;

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
        /*
        Controller 1 is main driver who controls movement, lift, and airplane
        Controller 2 controls intake and has control over dropping a pixel b/c missdrops happen way too often
        -Eliezer
         */
        while (opModeIsActive()) {
            TelemetryUpdate();
            while (!pullup) {
                double speed = 1;
                if (driveswitch == 0) {
                    speed = 1;
                } else if (driveswitch == 1) {
                    speed = .66;
                }
                if (gamepad2.dpad_left) { // P2 controls dropping b/c accedental drops are too common
                    DropServo.setPosition(.5);
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


                if (gamepad1.a && driveswitch == 1) { // P1 controls driving so they still have control over drive speed
                    driveswitch = 0;
                } else if (gamepad1.b && driveswitch == 0) {
                    driveswitch = 1;
                }
                if (gamepad2.y || (gamepad2.right_bumper && gamepad2.left_bumper)) { // P2 can take control of intake.
                    BackIntake.setPower(1);
                    MiddleIntake.setPower(-1);
                } else if (gamepad2.right_bumper) {
                    BackIntake.setPower(1);
                    MiddleIntake.setPower(0);
                } else if (gamepad2.left_bumper) {
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

                if (gamepad2.right_trigger > .1) { // P2 controls intake stuff
                    IntakeString.setPower(gamepad1.right_trigger);
                } else if (gamepad2.left_trigger > .1) {
                    IntakeString.setPower(-1 * gamepad1.left_trigger);
                } else {
                    IntakeString.setPower(0.0);
                }

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




