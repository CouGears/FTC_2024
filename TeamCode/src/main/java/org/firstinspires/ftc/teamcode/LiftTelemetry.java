package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class LiftTelemetry extends OpMode {

    private DcMotor Lift;
    private DistanceSensor BackdropDistance;

    @Override
    public void init() {
        // Initialize the motors. Make sure the names match your configuration
        Lift = hardwareMap.get(DcMotor.class, "Lift");
        BackdropDistance = hardwareMap.get(DistanceSensor.class, "BackdropDistance");

        // You can set other motor behaviors here if needed, like direction or mode
    }

    @Override
    public void loop() {
        // Read encoder values
        int liftPos = Lift.getCurrentPosition();
        double backdropDistancePos = BackdropDistance.getDistance(DistanceUnit.INCH);

        // Output encoder values to telemetry
        telemetry.addData("Lift: ", liftPos);
        telemetry.addData("Backdrop Distance: ", backdropDistancePos);
        telemetry.update();
    }
}
