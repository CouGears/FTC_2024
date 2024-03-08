package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.JoshAuton.RobotMethods;

@TeleOp
@Disabled
public class IntakeStringTesting extends OpMode {

    RobotMethods robot = new RobotMethods();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robot.setIntakeString(1);
        } else if (gamepad1.b) {
            robot.setIntakeString(-1);
        } else {
            robot.setIntakeString(0);
        }
    }
}
