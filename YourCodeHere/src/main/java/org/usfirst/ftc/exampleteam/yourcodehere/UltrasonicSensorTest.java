package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Robot starts on blue side, goes to beacon,
 * presses beacon button, and scores climber
 */
@TeleOp(name="UltrasonicSensorTest")
@Disabled
public class UltrasonicSensorTest extends SynchronousOpMode
{
    UltrasonicSensor ultrasonicSensor = null;

    @Override public void main() throws InterruptedException
    {
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonicSensor");

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
                telemetry.item("Level: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return ultrasonicSensor.getUltrasonicLevel();
                    }
                }));
        telemetry.addLine(
                telemetry.item("Status: ", new IFunc<Object>() {
                    @Override
                    public Object value() {
                        return ultrasonicSensor.status();
                    }
                }));

    }

}
