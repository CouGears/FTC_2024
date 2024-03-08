package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class DeadwheelTelemetry extends OpMode {

    private DcMotor middleWheel, rightWheel, leftWheel;

    @Override
    public void init() {
        // Initialize the motors. Make sure the names match your configuration
        middleWheel = hardwareMap.get(DcMotor.class, "BackIntake");
        rightWheel = hardwareMap.get(DcMotor.class, "PullUp");
        leftWheel = hardwareMap.get(DcMotor.class, "MiddleIntake");

        middleWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // You can set other motor behaviors here if needed, like direction or mode
    }

    @Override
    public void loop() {
        // Read encoder values
        int encoderMiddleWheel = middleWheel.getCurrentPosition();
        int encoderRightWheel = rightWheel.getCurrentPosition();
        int encoderLeftWheel = leftWheel.getCurrentPosition();

        // Output encoder values to telemetry
        telemetry.addData("Middle", encoderMiddleWheel);
        telemetry.addData("Right", encoderRightWheel);
        telemetry.addData("Left", encoderLeftWheel);
        telemetry.update();
    }
}
