package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OneFootTests extends LinearOpMode {

    private AutonMethods robot = new AutonMethods();
    private String currentMode = ""; // This will keep track of the current sequence
    private int step = 0; // This will keep track of the steps within a sequence
    private double gspeed = .5;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry, false);
        waitForStart();

        while (opModeIsActive()) {
            // Check if a sequence is currently running or if a button is pressed to start a sequence
            if (currentMode.equals("") && gamepad1.a) {
                currentMode = "A";
            } else if (currentMode.equals("") && gamepad1.b) {
                currentMode = "B";
            } else if (currentMode.equals("") && gamepad1.x) {
                currentMode = "X";
            } else if (currentMode.equals("") && gamepad1.y) {
                currentMode = "Y";
            }

            switch (currentMode) {
                case "A":
                    // Execute the next step in the A sequence
                    if (step == 0) {
                        robot.drive(2 * robot.feet, 0, gspeed);
                    } else if (step == 1) {
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                case "B":
                    // Execute the next step in the A sequence
                    if (step == 0) {
                        robot.drive(-2 * robot.feet, 0, gspeed);
                    } else if (step == 1) {
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                case "X":
                    // Execute the next step in the A sequence
                    if (step == 0) {
                        robot.drive(0 * robot.feet, 2 * robot.feet, gspeed);
                    } else if (step == 1) {
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                case "Y":
                    // Execute the next step in the A sequence
                    if (step == 0) {
                        robot.drive(0 * robot.feet, -2 * robot.feet, gspeed);
                    } else if (step == 1) {
                        currentMode = ""; // Sequence finished, reset mode
                        step = -1; // Reset step counter
                    }
                    step++;
                    break;

                default:
                    break;
            }
            sleep(3000);
            telemetry.update();
        }
    }
}
