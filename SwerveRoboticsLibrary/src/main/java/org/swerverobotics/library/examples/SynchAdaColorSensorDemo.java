package org.swerverobotics.library.examples;

import com.qualcomm.hardware.AdafruitI2cColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * SynchLEDDemo is a short demo on using the Adafruit Color Sensor module.
 * It assumes that you have an Adafruit Color Sensor connected to a Core Device Interface
 * called "color". We don't really need to control the sensor's LED - it comes on by default.
 */
@TeleOp(name="AdaColorSensor", group="Swerve Examples")
public class SynchAdaColorSensorDemo extends SynchronousOpMode {

    ColorSensor sensorRGB = null;

    @Override public void main() throws InterruptedException {

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("color");

        // Set up our dashboard computations
        composeDashboard();

        // Wait until we're told to go
        waitForStart();
               
        // Loop and update the dashboard
        while (this.opModeIsActive()) {

            telemetry.update();

            idle();
        }
    }
    
    void composeDashboard() {
        telemetry.addLine(
                telemetry.item("loop count: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return getLoopCount();
                    }
                }));
        telemetry.addLine(
                telemetry.item("color: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                //return led.getState(); //unlike DigitalChannel, leds don't let you read their state, so use the variable here instead.
                                return "r: " + sensorRGB.red() + " g: " + sensorRGB.green() + " b: " + sensorRGB.blue();
                            }
                        })
        );
    }

}
