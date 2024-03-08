package org.firstinspires.ftc.teamcode.JoshNewAuton;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.JoshAuton.RobotMethods;

public class BaseMethods {
    public Telemetry telemetry;
    public RobotMethods robot;
    public BaseMethods(Telemetry telemetry, RobotMethods robot) {
        this.telemetry = telemetry;
        this.robot = robot;
    }
    public void placePixel() {
        robot.moveLift(400, 1);
        sleep(500);
        robot.setDropServo(0.5);
        sleep(500);
        robot.setDropServo(0.045);
        sleep(200);
        robot.moveLift(-400, 1);
    }

    public void spitPixel() {
        robot.moveLift(400, 1);
        sleep(500);
        robot.setDropServo(0.5);
        sleep(500);
        robot.setDropServo(0.045);
        sleep(200);
        robot.moveLift(-400, 1);
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
