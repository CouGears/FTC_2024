package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutonMethods{

    //Constructor
    public AutonMethods() {

    }

    //Declare and initial variables
    private double rev = 537.7;//revolution of 312 rpm motor , find at https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-6mm-d-shaft-312-rpm-36mm-gearbox-3-3-5v-encoder/  called encoder resolution
    private double pi = 3.14;
    private double  wheelDiameter = 3.77953;//inch
    private double robotWidth = 14;//inch
    private double robotLength = 17;//inch
    private double CalebTurnConstant = 1.54;
    private double CalebDistanceConstant = 2;
    private double CalebSideConstant = CalebDistanceConstant*1.11;


    private double circumscribedDiameter = Math.sqrt(Math.pow(robotLength, 2) + Math.pow(robotWidth, 2));//inch
    private double circumscribedRadius = circumscribedDiameter / 2;//inch
    private double inch = rev / (wheelDiameter * pi);
    public double feet = inch * 12;
    private double rev2 = 2048;//revolution of 435 rpm motor
    private double inch2 = rev2 / (2 * pi);
    private double feet2 = inch2 * 12;

    private int topLiftEncoder = 7475;
    private double botR = 1;
    private double topR = 0;
    private double botL = .35;
    private double topL = 0;
    public static DcMotor motorBR, motorBL, motorFL, motorFR, BackIntake, MiddleIntake, Lift;
    public static CRServo IntakeString;
    public static Servo DropServo;
    private DistanceSensor LeftDistance, RightDistance;
    private final ElapsedTime runtime = new ElapsedTime();
    public static int Case = 0;
    HardwareMap map;
    Telemetry tele;
    public int counter = 0;

    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    public void initBasic(HardwareMap map, Telemetry tele) {
        this.tele = tele;
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");

        motorFL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

    }
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        this.tele = tele;
        motorFL = map.get(DcMotor.class, "motorFL");
        motorBL = map.get(DcMotor.class, "motorBL");
        motorBR = map.get(DcMotor.class, "motorBR");
        motorFR = map.get(DcMotor.class, "motorFR");
        BackIntake = map.get(DcMotor.class, "BackIntake");
        MiddleIntake = map.get(DcMotor.class, "MiddleIntake");
        Lift = map.get(DcMotor.class, "Lift");
        IntakeString = map.get(CRServo.class, "IntakeString");
        DropServo = map.get(Servo.class, "DropServo");
        LeftDistance = map.get(DistanceSensor.class, "LeftDistance");
        RightDistance = map.get(DistanceSensor.class, "RightDistance");


        motorFL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        BackIntake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        MiddleIntake.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        Lift.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBR.setDirection(DcMotorSimple.Direction.FORWARD);
        BackIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        MiddleIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFL.setTargetPosition(0);
        motorBL.setTargetPosition(0);
        motorFR.setTargetPosition(0);
        motorBR.setTargetPosition(0);
        BackIntake.setTargetPosition(0);
        MiddleIntake.setTargetPosition(0);

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

         this.tele.addData(">", "Gyro Calibrating. Do Not Move!");
        this.tele.update();
    }

    public void kill() {
        motorFL.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
        MiddleIntake.setPower(0);
        BackIntake.setPower(0);
        Lift.setPower(0);

    }

    public double maps(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    //Function to move the robot in any direction
    public void drive(double forward, double sideways, double speed) {
        runtime.reset();
        forward*=CalebDistanceConstant;
        sideways*=(CalebSideConstant);
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(RunMode.STOP_AND_RESET_ENCODER);

        double FRtpos = forward - sideways;
        double BRtpos = forward + sideways;
        double FLtpos = forward + sideways;
        double BLtpos = forward - sideways;

        motorFL.setTargetPosition(-(int) FLtpos);
        motorBL.setTargetPosition((int) BLtpos);
        motorFR.setTargetPosition(-(int) FRtpos);
        motorBR.setTargetPosition((int) BRtpos);

        motorFL.setMode(RunMode.RUN_TO_POSITION);
        motorBL.setMode(RunMode.RUN_TO_POSITION);
        motorFR.setMode(RunMode.RUN_TO_POSITION);
        motorBR.setMode(RunMode.RUN_TO_POSITION);

        speed(speed);
    }

    public void speed(double speed) {
        motorFL.setPower(speed);
        motorBL.setPower(speed);
        motorFR.setPower(speed);
        motorBR.setPower(speed);
    }


    //circumscribed robot has a diameter of 21 inches
    public void turn(double deg) {
        while (motorFR.isBusy() || motorFL.isBusy()) {
            if (runtime.seconds() > 2) break;
        }
        motorFL.setMode(RunMode.STOP_AND_RESET_ENCODER); //for every drive function remember to reset encoder
        motorBL.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(RunMode.STOP_AND_RESET_ENCODER);
        double deltaturn = (deg / 360) * 7750;

        motorFL.setTargetPosition(-(int) deltaturn);
        motorBL.setTargetPosition((int) deltaturn);
        motorFR.setTargetPosition((int) deltaturn);
        motorBR.setTargetPosition(-(int) deltaturn);
        motorFR.setMode(RunMode.RUN_TO_POSITION);
        motorBR.setMode(RunMode.RUN_TO_POSITION);
        motorFL.setMode(RunMode.RUN_TO_POSITION);
        motorBL.setMode(RunMode.RUN_TO_POSITION);
        motorFL.setPower(0.5);
        motorBL.setPower(0.5);
        motorFR.setPower(0.5);
        motorBR.setPower(0.5);

    }

    public void DropSetPosition(double deg) {
        DropServo.setPosition(deg);
    }

    public double GetLeftDistance(){
        return LeftDistance.getDistance(DistanceUnit.CM);
    }

    public double GetRightDistance(){
        return RightDistance.getDistance(DistanceUnit.CM);
    }


    public void LiftSetPosition(int targetPosition) {

        int errorMargin = 10; // Acceptable range to consider the target reached

        // Basic motor setup
        Lift.setMode(RunMode.RUN_USING_ENCODER);

        int currentPosition = Lift.getCurrentPosition(); // Get the current position

        // Calculate the direction to move
        int direction = targetPosition > currentPosition ? 1 : -1;
        // Set a constant power level for movement (adjust as necessary)
        double power = -0.006 * direction;

        while (Math.abs(Lift.getCurrentPosition() - targetPosition) > errorMargin) {
            if(Math.abs(Lift.getCurrentPosition())>Math.abs(targetPosition)){
                Lift.setPower(0);
                return;
            }
            Lift.setPower(power);
        }

        // Stop the motor once the target is reached
        Lift.setPower(0);
        tele.addData("Status", "Target Reached");
        tele.update();
    }








    public void IntakeString(double speed) {
        IntakeString.setPower(speed);
    }
    public void MiddleIntakeSpeed(double speed){
        MiddleIntake.setPower(-1*speed);
    }


//


    //Function to have the robot sleep
    public void sleep(long sleep) {
        try {
            Thread.sleep(sleep);
        } catch (InterruptedException e) {
            tele.addLine("Failed Sleep");
            tele.update();
        }
    }


}