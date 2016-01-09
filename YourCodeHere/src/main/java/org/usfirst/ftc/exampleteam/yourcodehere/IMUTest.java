package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Testing IMU's and having fun doing it
 */
@TeleOp(name="IMUTest")
public class IMUTest extends Master8923TeleOp
{
    IBNO055IMU imu;
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    double currentHeading;

    @Override protected void main() throws InterruptedException
    {
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        imu                       = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        configDash();
        telemetry.setUpdateIntervalMs(30);

        waitForStart();

        double calibratedHeading = imu.getAngularOrientation().heading;

        while(opModeIsActive())
        {
            currentHeading = imu.getAngularOrientation().heading - calibratedHeading;

            if(currentHeading < 0)
                currentHeading += 360;

            telemetry.update();
            idle();
        }
    }

    public void configDash()
    {
        telemetry.addLine(
            telemetry.item("heading: ", new IFunc<Object>()
            {
                public Object value()
                {
                    return currentHeading;
                }
            }));

    }
}