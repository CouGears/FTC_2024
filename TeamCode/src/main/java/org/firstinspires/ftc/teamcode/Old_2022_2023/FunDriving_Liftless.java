package org.firstinspires.ftc.teamcode.Old_2022_2023;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.AutonMethods;
import org.firstinspires.ftc.teamcode.SensorSet.LEDMethods;

//TODO: Uncomment the following line to use
@TeleOp
@Disabled

public class FunDriving_Liftless extends LinearOpMode {

    public static DcMotor motorBR, motorBL, motorFL, motorFR, LiftRight, LiftLeft;
    public static Servo intake, armL, armR;

    private AutonMethods robot = new AutonMethods();
    public int driveswitch = 1;
    public double armPos = .5;
    public boolean binaryarmmovement = true;
    public double OtherarmPos = .5;
    private int topLiftEncoder = 7475;
    private double botR = 1;
    private double topR = 0;
    private double botL = .35;
    private double topL = 0;
    private double right = 0;
    private double left = 0;

    public void TelemetryUpdate() {
        telemetry.addData("Drive Mode", driveswitch);
        telemetry.addLine();
        telemetry.addData("Binary Arm Movememt", binaryarmmovement);
        telemetry.addLine();
        telemetry.addData("armPos - Variable", armPos);
        telemetry.addLine();
        telemetry.addData("Left - Servo Actual", armL.getPosition());
        telemetry.addLine();
        telemetry.addData("Right - Servo Actual", armR.getPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        //region hardware map
        LEDMethods LED = new LEDMethods();
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        LiftLeft = hardwareMap.get(DcMotor.class, "LiftLeft");
        LiftRight = hardwareMap.get(DcMotor.class, "LiftRight");

        intake = hardwareMap.get(Servo.class, "intake");
        armL = hardwareMap.get(Servo.class, "armL");
        armR = hardwareMap.get(Servo.class, "armR");

        LiftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LiftRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LiftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        LiftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        LiftRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            TelemetryUpdate();
            double speed = 1;
            if (driveswitch == 0) {
                speed = .333;
            } else if (driveswitch == 1) {
                speed = .666;
            } else if (driveswitch == 2) {
                speed = 1;
            }

            motorFL.setPower(((this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) + ((this.gamepad1.left_stick_y)) - (this.gamepad1.left_stick_x)) * speed);
            motorBL.setPower(-(-(this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) - (this.gamepad1.left_stick_x)) * speed);
            motorBR.setPower((-(this.gamepad1.right_stick_y) - (this.gamepad1.right_stick_x) - (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * speed);
            motorFR.setPower(-((this.gamepad1.right_stick_y) + (this.gamepad1.right_stick_x) + (this.gamepad1.left_stick_y) + (this.gamepad1.left_stick_x)) * speed);
            if (gamepad1.a) {
                driveswitch = 2;
            } else if (gamepad1.x) {
                driveswitch = 1;
            } else if (gamepad1.y) {
                driveswitch = 0;
            }
            if (gamepad1.right_bumper) {
                binaryarmmovement = false;
            } else if (gamepad1.left_bumper) {
                binaryarmmovement = true;
            }
            if (gamepad1.dpad_up) {
                //if(LiftRight.getCurrentPosition() <= topLiftEncoder) {
                if (binaryarmmovement == false && armPos <= 1) {
                    armPos += .01;
                    OtherarmPos = 1 - armPos;
                } else {
                    armPos = 1;
                    OtherarmPos = 0;
                }
                armL.setPosition(armPos);
                armR.setPosition(OtherarmPos);

                //}
            } else if (gamepad1.dpad_down) {
                if (binaryarmmovement == false && armPos >= 0) {
                    armPos -= .01;
                    OtherarmPos = 1 - armPos;
                } else {
                    armPos = 0;
                    OtherarmPos = 1;
                }
                armL.setPosition(armPos);
                armR.setPosition(OtherarmPos);
            } else {
                LiftLeft.setPower(0);
                LiftRight.setPower(0);
            }
            if (gamepad1.right_trigger > 0.5) {
                intake.setPosition(.35);
            } else if (gamepad1.left_trigger > 0.5) {
                intake.setPosition(0);
            }
        }
    }
}





