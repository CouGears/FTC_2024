package org.firstinspires.ftc.teamcode.JoshAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
@Disabled
public class MethodsTest extends OpMode {

    RobotMethods robot = new RobotMethods();

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {

        robot.drive(24, 0, 1);
        robot.returnAfterBusy();
        robot.drive(0, 24, 1);
        robot.returnAfterBusy();
        robot.drive(-24, 0, 1);
        robot.returnAfterBusy();
        robot.drive(0, -24, 1);
        robot.returnAfterBusy();
        robot.drive(24, 0, 1);
        robot.returnAfterBusy();
        robot.turn(90, 1);
        robot.returnAfterBusy();
        robot.drive(24, 0, 1);
        robot.returnAfterBusy();
        robot.turn(90, 1);
        robot.returnAfterBusy();
        robot.drive(24, 0, 1);
        robot.returnAfterBusy();
        robot.turn(90, 1);
        robot.returnAfterBusy();
        robot.drive(24, 0, 1);
        robot.returnAfterBusy();
        robot.turn(90, 1);
        robot.returnAfterBusy();
        robot.drive(24, 24, 1);
        robot.returnAfterBusy();
        robot.drive(-24, 0, 1);
        robot.returnAfterBusy();
        robot.drive(24, -24, 1);
        robot.returnAfterBusy();
        robot.drive(-24, 0, 1);
        robot.returnAfterBusy();

        sleep(10000);

    }

    public void sleep(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            telemetry.addLine("Failed Sleep");
            telemetry.update();
        }
    }

}
