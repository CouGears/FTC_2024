package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//TODO: Uncomment the following line to use
@TeleOp
public class ZellMotor extends LinearOpMode {

    public static DcMotor motorZell;
    private AutonMethods robot = new AutonMethods();
    public int driveswitch = 1;

    public int intakemode = 0;
    private int liftLimit = -3000;


    @Override
    public void runOpMode() {
        //region hardware map
        motorZell = hardwareMap.get(DcMotor.class, "motorZell");

        motorZell.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motorZell.setDirection(DcMotorSimple.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            motorZell.setPower(this.gamepad1.left_stick_y);
            }
        }
    }




