package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Robot starts on blue side, goes to beacon,
 * presses beacon button, and scores climber
 */
@Autonomous(name = "ColorSensorTest")
@Disabled
public class ColorSensorTest extends SynchronousOpMode {
    ColorSensor sensorRGB = null;
    DigitalChannel sensorLED = null;

    final boolean LED_ON = true;
    final boolean LED_OFF = false;

    @Override
    public void main() throws InterruptedException {

        // get a reference to our ColorSensor object.
        sensorRGB = hardwareMap.colorSensor.get("colorSensorBeacon");

        // get a reference to the LED on the color sensor board
        sensorLED = hardwareMap.digitalChannel.get("LED");
        sensorLED.setMode(DigitalChannelController.Mode.OUTPUT);
        sensorLED.setState(LED_OFF);

        // Set up our dashboard computations
        composeDashboard();

        // Wait until we're told to go
        waitForStart();

        // Loop and update the dashboard
        while (this.opModeIsActive()) {

            if (this.updateGamepads()) {
                if (this.gamepad1.a) {
                    //toggle LED state
                    sensorLED.setState(!sensorLED.getState());
                }
            }

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
                        return "r: " + sensorRGB.red() + " g: " + sensorRGB.green() + " b: " + sensorRGB.blue();
                    }
                }));
        telemetry.addLine(
                telemetry.item("led: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return sensorLED.getState();
                    }
                }));
        telemetry.addLine(
                telemetry.item("Compiled Color: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        if (sensorRGB.blue() >= sensorRGB.red() && sensorRGB.blue() >= sensorRGB.green()) {
                            return "Blue";
                        } else if (sensorRGB.red() >= sensorRGB.blue() && sensorRGB.red() >= sensorRGB.green()) {
                            return "Red";
                        } else {
                            return "White or Black";
                        }
                    }
                }));

    }

}


