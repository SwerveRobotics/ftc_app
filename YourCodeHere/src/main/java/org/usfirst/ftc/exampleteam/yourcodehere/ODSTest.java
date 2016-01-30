package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Robot starts on blue side, goes to beacon,
 * presses beacon button, and scores climber
 */
@TeleOp(name="ODS Test")
@Disabled
public class ODSTest extends SynchronousOpMode
{
    OpticalDistanceSensor lightSensorFront = null;
    OpticalDistanceSensor lightSensorBack = null;

    @Override public void main() throws InterruptedException
    {
        lightSensorFront = hardwareMap.opticalDistanceSensor.get("lightSensorFront");
        lightSensorBack = hardwareMap.opticalDistanceSensor.get("lightSensorBack");
        lightSensorFront.enableLed(true);
        lightSensorBack.enableLed(true);

        // Set up our dashboard computations
        composeDashboard();

        waitForStart();

        while(opModeIsActive())
        {
            telemetry.update();
            idle();
        }
    }

    void composeDashboard() {
        telemetry.addLine(
                telemetry.item("Front: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return formatNumber(lightSensorFront.getLightDetected());
                    }
                }),
                this.telemetry.item("Back: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return formatNumber(lightSensorBack.getLightDetected());
                    }
                }));

    }

    public String formatNumber(double number)
    {
        return String.format("%.2f", number);
    }

}
