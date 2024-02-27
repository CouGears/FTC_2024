package org.firstinspires.ftc.teamcode.SensorSet;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class AutonMethodsNew {
    LimitMethods Limit = new LimitMethods();
    LEDMethods LED = new LEDMethods();
    DistanceMethods Distance = new DistanceMethods();
    MagnetMethods Magnet = new MagnetMethods();

    //Constructor
    public AutonMethodsNew() {

    }

    //Declare and initial variables
    private ElapsedTime runtime = new ElapsedTime();
    HardwareMap map;
    Telemetry tele;

    public static BNO055IMU imu;
    BNO055IMU.Parameters parameters;
    Orientation angles;

    //Initialization
    public void init(HardwareMap map, Telemetry tele, boolean auton) {

        int relativeLayoutId = map.appContext.getResources().getIdentifier("RelativeLayout", "id", map.appContext.getPackageName());

    }


}
