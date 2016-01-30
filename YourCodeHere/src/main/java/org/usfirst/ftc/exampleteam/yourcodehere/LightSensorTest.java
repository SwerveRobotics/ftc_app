package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Robot starts on blue side, goes to beacon,
 * presses beacon button, and scores climber
 */
@TeleOp(name="LightSensorTest")
@Disabled
public class LightSensorTest extends SynchronousOpMode
{
    LightSensor lightSensorFront = null;
    LightSensor lightSensorBack = null;

    boolean toggle = true;

    @Override public void main() throws InterruptedException
    {
        lightSensorFront = hardwareMap.lightSensor.get("lightSensorFront");
        lightSensorBack = hardwareMap.lightSensor.get("lightSensorBack");

        // Set up our dashboard computations
        composeDashboard();

        lightSensorBack.enableLed(toggle);
        lightSensorFront.enableLed(toggle);

        waitForStart();

        while(opModeIsActive())
        {
            if(updateGamepads())
            {
                if(gamepad1.a)
                {
                    toggle = ! toggle;
                    lightSensorBack.enableLed(toggle);
                    lightSensorFront.enableLed(toggle);
                }
            }
            telemetry.update();
            idle();
        }
    }

    void composeDashboard() {
        telemetry.addLine(
                telemetry.item("Back: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return lightSensorBack.getLightDetected();
                    }
                }));
        telemetry.addLine(
                telemetry.item("Front: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return lightSensorFront.getLightDetected();
                    }
                }));

    }

}
