package org.usfirst.ftc.exampleteam.yourcodehere;

import android.content.SharedPreferences;

import com.qualcomm.robotcore.hardware.ColorSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;


/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@Autonomous(name="ColorSensorCalibration")
@Disabled
public class ColorSensorCalibration extends SynchronousOpMode {

    ColorSensor colorSensorBeacon;
    @Override
    public void main() throws InterruptedException {
        colorSensorBeacon = hardwareMap.colorSensor.get("colorSensorBeacon");

        waitForStart();

        while(opModeIsActive())
        {
            if(updateGamepads())
            {
                if (gamepad1.x)
                {
                    calibrateBlue();
                    telemetry.addData("Blue is ", "Calibrated!");
                }
                if(gamepad1.b)
                {
                    calibrateRed();
                    telemetry.addData("Red is ", "Calibrated!");
                }
            }
        }

    }

    public int calibrateBlue() {
        int blue;
        blue = colorSensorBeacon.blue();
        telemetry.log.add("blue: " + blue);
        return blue;
    }

    public int calibrateRed() {
        int red;
        red = colorSensorBeacon.red();
        telemetry.log.add("red: " + red);
        return red;
    }
}

