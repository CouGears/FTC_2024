package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class EncoderTelemetry extends OpMode {

    private DcMotor motorFL, motorBL, motorFR, motorBR;

    @Override
    public void init() {
        // Initialize the motors. Make sure the names match your configuration
        motorFL = hardwareMap.get(DcMotor.class, "motorFL");
        motorBL = hardwareMap.get(DcMotor.class, "motorBL");
        motorFR = hardwareMap.get(DcMotor.class, "motorFR");
        motorBR = hardwareMap.get(DcMotor.class, "motorBR");

        // You can set other motor behaviors here if needed, like direction or mode
    }

    @Override
    public void loop() {
        // Read encoder values
        int encoderFL = motorFL.getCurrentPosition();
        int encoderBL = motorBL.getCurrentPosition();
        int encoderFR = motorFR.getCurrentPosition();
        int encoderBR = motorBR.getCurrentPosition();

        // Output encoder values to telemetry
        telemetry.addData("Encoder FL", encoderFL);
        telemetry.addData("Encoder BL", encoderBL);
        telemetry.addData("Encoder FR", encoderFR);
        telemetry.addData("Encoder BR", encoderBR);
        telemetry.update();
    }
}
