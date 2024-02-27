package org.firstinspires.ftc.teamcode.JoshAuton;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
// BASIC METHODS FOR AUTONOMOUS ROBOT MOVEMENT AND CONTROL
public class RobotMethods {

    // Motor encoder units per full revolution
    private double revolution = 1200;

    // Pi to 2 decimal places
    private double pi = 3.14;
    // Wheel diameter in inches
    private double wheelDiameter = 3.77953;
    // Wheel circumference in inches
    private double wheelCircumference = wheelDiameter * pi;
    // Inches per full wheel revolution
    private double inchesPerRevolution = wheelCircumference;
    //
    private double revolutionsPerInch = 1 / inchesPerRevolution;

    private double inch = revolution * revolutionsPerInch;
    private double foot = inch * 12;

    private double fwdErrorCorrection = 0.9;
    private double sideErrorCorrection = 1.05;

    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift;
    public static CRServo IntakeString;
    public static Servo DropServo;
    //private DistanceSensor LeftDistance, RightDistance;
    public static DistanceSensor BackdropDistance;

    public void init(HardwareMap map, Telemetry tele) {
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        BackIntake = map.get(DcMotor.class, "BackIntake");
        MiddleIntake = map.get(DcMotor.class, "MiddleIntake");
        Lift = map.get(DcMotor.class, "Lift");
        IntakeString = map.get(CRServo.class, "IntakeString");
        DropServo = map.get(Servo.class, "DropServo");
        //LeftDistance = map.get(DistanceSensor.class, "LeftDistance");
        //RightDistance = map.get(DistanceSensor.class, "RightDistance");
        BackdropDistance = map.get(DistanceSensor.class, "BackdropDistance");

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MiddleIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        MiddleIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        BackIntake.setTargetPosition(0);
        MiddleIntake.setTargetPosition(0);
        Lift.setTargetPosition(0);

        DropServo.setPosition(.045);
    }

    public void drive(double fwd, double side, double speed) {
        fwd *= inch;
        side *= inch;
        fwd *= fwdErrorCorrection;
        side *= sideErrorCorrection;

//        double fwd2 = fwd * fwd;
//        double side2 = side * side;
//
//        double FRtarget = Math.sqrt(Math.abs(fwd2 - side2));
//        double BRtarget = Math.sqrt(Math.abs(fwd2 + side2));
//        double FLtarget = Math.sqrt(Math.abs(-fwd2 - side2));
//        double BLtarget = Math.sqrt(Math.abs(-fwd2 + side2));

//        if (fwd - side < 0) {
//            FRtarget *= -1;
//        }
//        if (fwd + side < 0) {
//            BRtarget *= -1;
//        }
//        if (-fwd - side < 0) {
//            FLtarget *= -1;
//        }
//        if (-fwd + side < 0) {
//            BLtarget *= -1;
//        }

        double FRtarget = fwd - side;
        double BRtarget = fwd + side;
        double FLtarget = -fwd - side;
        double BLtarget = -fwd + side;

        double FRspeed = speed, BRspeed = speed, FLspeed = speed, BLspeed = speed;
        if (Math.abs(FRtarget) > Math.abs(FLtarget)) {
            FLspeed = Math.abs((FLtarget / FRtarget)) * speed;
            BRspeed = FLspeed;
        } else if (Math.abs(FRtarget) < Math.abs(FLtarget)) {
            FRspeed = Math.abs((FRtarget / FLtarget)) * speed;
            BLspeed = FRspeed;
        }

        wheelMode(0);
        wheelTarget(FRtarget, BRtarget, FLtarget, BLtarget);
        wheelMode(1);
        wheelSpeed(FRspeed, BRspeed, FLspeed, BLspeed);
    }

    public void turn(double deg, double speed) {
        double revolutions = deg / 360;
        double turn = revolutions * -7750;

        wheelMode(0);
        wheelTarget(turn);
        wheelMode(1);
        wheelSpeed(speed);
    }

    public void wheelMode(int mode) {
        switch (mode) {
            case 0:
                motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                break;
            case 1:
                motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void wheelSpeed(double spd) {
        motorFR.setPower(spd);
        motorBR.setPower(spd);
        motorFL.setPower(spd);
        motorBL.setPower(spd);
    }

    public void wheelSpeed(double FR, double BR, double FL, double BL) {
        motorFR.setPower(FR);
        motorBR.setPower(BR);
        motorFL.setPower(FL);
        motorBL.setPower(BL);
    }

    public void wheelTarget(double target) {
        motorFR.setTargetPosition((int) target);
        motorBR.setTargetPosition((int) target);
        motorFL.setTargetPosition((int) target);
        motorBL.setTargetPosition((int) target);
    }

    public void wheelTarget(double FR, double BR, double FL, double BL) {
        motorFR.setTargetPosition((int) FR);
        motorBR.setTargetPosition((int) BR);
        motorFL.setTargetPosition((int) FL);
        motorBL.setTargetPosition((int) BL);
    }

    public void stopWheels() {
        wheelSpeed(0.0);
        wheelMode(0);
        wheelTarget(0.0);
    }

    public boolean wheelsBusy() {
        if (motorFR.isBusy() || motorBR.isBusy() || motorFL.isBusy() || motorBL.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean motorsBusy() {
        if (wheelsBusy() || BackIntake.isBusy() || MiddleIntake.isBusy() || Lift.isBusy()) {
            return true;
        } else {
            return false;
        }
    }

    public boolean returnAfterBusy() {
        while (true) {
            if (!motorsBusy()) {
                return true;
            }
        }
    }

//    public double getDistance(int sensor) {
//        switch (sensor) {
//            case 0:
//                return LeftDistance.getDistance(DistanceUnit.INCH);
//            case 1:
//                return RightDistance.getDistance(DistanceUnit.INCH);
//            default:
//                return 0.0;
//        }
//    }

    public double getLift() {
        return -Lift.getCurrentPosition();
    }

    public double getDrop() {
        return DropServo.getPosition();
    }

    public void moveLift(int target, double pwr, Telemetry tele) {
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(target-getLift()) > 10) {
            if (target > getLift()) {
                Lift.setPower(-pwr);
            } else if (target < getLift()) {
                Lift.setPower(pwr);
            }
            tele.addLine("Lift: " + getLift());
            tele.update();
        }
        Lift.setPower(0);
    }

    public void middle(double power) {
        MiddleIntake.setPower(power);
    }

    public void setDropServo(double pos) {
        DropServo.setPosition(pos);
    }

    public void backIntake(double power) {
        BackIntake.setPower(power);
    }

    public double getBackdropDistance() {
        return BackdropDistance.getDistance(DistanceUnit.INCH);
    }


}
