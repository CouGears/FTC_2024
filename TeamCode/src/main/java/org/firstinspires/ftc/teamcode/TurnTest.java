package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.JoshAuton.RobotMethods;

@TeleOp
public class TurnTest extends LinearOpMode {

    public static DcMotor motorBR, motorBL, motorFL, motorFR;

    public RobotMethods robot = new RobotMethods();

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {

            robot.init(hardwareMap, telemetry);

//            motorFL = hardwareMap.get(DcMotor.class, "motorFL");
//            motorBL = hardwareMap.get(DcMotor.class, "motorBL");
//            motorBR = hardwareMap.get(DcMotor.class, "motorBR");
//            motorFR = hardwareMap.get(DcMotor.class, "motorFR");
//
//            motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//            motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
//            motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
//            motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
//            motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

//            motorFL.setTargetPosition(0);
//            motorBL.setTargetPosition(0);
//            motorFR.setTargetPosition(0);
//            motorBR.setTargetPosition(0);

            while (!gamepad1.a) {
            }
//            motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //for every drive function remember to reset encoder
//            motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            // double deltaturn = (1 / (3.77953 * 3.14)) * 24 * .9 * 1200; // fwd
//            double deltaturn = (1 / (3.77953 * 3.14)) * 24 * 1.05 * 1200; // side
//
//            motorFL.setTargetPosition(-(int) deltaturn);
//            motorBL.setTargetPosition((int) deltaturn);
//            motorFR.setTargetPosition(-(int) deltaturn);
//            motorBR.setTargetPosition((int) deltaturn);
//            motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            motorFL.setPower(0.5);
//            motorBL.setPower(0.5);
//            motorFR.setPower(0.5);
//            motorBR.setPower(0.5);
            robot.drive(24, 0, 0.5);
            sleep(7000);
            robot.drive(0, 24, 0.5);
            sleep(7000);
            robot.drive(-24, 0, 0.5);
            sleep(7000);
            robot.drive(0, -24, 0.5);
            sleep(7000);

        }
    }

    public void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {

        }
    }
}
