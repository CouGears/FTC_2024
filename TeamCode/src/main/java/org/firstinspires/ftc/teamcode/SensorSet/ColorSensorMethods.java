package org.firstinspires.ftc.teamcode.SensorSet;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorMethods {
    public static ColorSensor color;

    //Constructor
    public ColorSensorMethods() {

    }

    private final ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry telemetry;
    public int match;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {
        color = map.get(ColorSensor.class, "Color");
        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());
    }

    public int sense(int red, int green, int blue, int marginRed, int marginGreen, int marginBlue) {
        if (color.red() <= red + marginRed && color.red() >= red - marginRed && color.green() <= green + marginGreen && color.green() >= green - marginGreen && color.blue() <= blue + marginBlue && color.blue() >= blue - marginBlue) {
            match = 1;
            return 1;
        }
        else
        {
            match = 0;
            return 0;
        }
    }
}