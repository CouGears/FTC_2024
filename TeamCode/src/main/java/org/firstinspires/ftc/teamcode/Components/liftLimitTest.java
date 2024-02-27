package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.AutonMethods;

//TODO: Uncomment the following line to use
@TeleOp
public class liftLimitTest extends LinearOpMode {

    public static DcMotor Lift;
    private AutonMethods robot = new AutonMethods();
    private int liftLimit = 5500;
    @Override
    public void runOpMode() {
        Lift = hardwareMap.get(DcMotor.class, "Lift");

        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.dpad_up && Lift.getCurrentPosition() <= liftLimit) {
                Lift.setPower(1);
            } else if (gamepad1.dpad_down && Lift.getCurrentPosition() >= 500){
                Lift.setPower(-1);
            } else {
                Lift.setPower(0);
            }
            telemetry.addData("Lift Pos = ", Lift.getCurrentPosition());
            telemetry.update();
        }
    }
}





