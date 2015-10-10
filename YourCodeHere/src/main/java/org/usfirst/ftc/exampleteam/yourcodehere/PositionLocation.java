package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * SynchIMUDemo gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 * http://www.adafruit.com/products/2472
 */
@TeleOp(name="rizz.i/o-kasinski", group="Razzle Examples")
public class PositionLocation extends SynchronousOpMode
{
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // Our sensors, motors, and other devices go here, along with other long term state
    IBNO055IMU              imu;
    Acceleration            acceleration;
    ElapsedTime             elapsed    = new ElapsedTime();
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();


    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int                     loopCycles;
    int                     i2cCycles;
    double                  ms;
    float prevXvel;
    float prevYvel;
    float prevZvel;
    float timeInterval;
    float totalXdisplacement;
    float totalYdisplacement;
    float totalZdisplacement;
    float correctedaX;
    float correctedaY;
    float correctedaZ;
    boolean firstRun;

    final double GRAPH_H_VALUE;
    final double GRAPH_K_VALUE;

    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorRearLeft;
    DcMotor motorRearRight;

    public PositionLocation() {
        GRAPH_H_VALUE = 2.15;
        GRAPH_K_VALUE = Math.log((2*GRAPH_H_VALUE/(2+GRAPH_H_VALUE))-1);
    }

    //----------------------------------------------------------------------------------------------
    // main() loop
    //----------------------------------------------------------------------------------------------

    @Override public void main() throws InterruptedException
    {
        // We are expecting the IMU to be attached to an I2C port on  a core device interface 
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        acceleration = imu.getLinearAcceleration();
        prevXvel = 0;
        prevYvel = 0;
        prevZvel = 0;
        timeInterval = 0.066666666666f;
        totalXdisplacement = 0;
        totalYdisplacement = 0;
        totalZdisplacement = 0;
        loopCycles = 0;


        motorFrontLeft = hardwareMap.dcMotor.get("fl");
        motorFrontRight = hardwareMap.dcMotor.get("fr");
        motorRearLeft = hardwareMap.dcMotor.get("rl");
        motorRearRight = hardwareMap.dcMotor.get("rr");

        if(Math.abs(acceleration.accelX) < 0.1 || loopCycles < 1000){
            correctedaX = 0;
        }
        else{
            correctedaX =(float) acceleration.accelX;
        }

        if(Math.abs(acceleration.accelY) < 0.1  || loopCycles < 1000)
        {
            correctedaY = 0;
        }
        else
        {
            correctedaY =(float) acceleration.accelY ;
        }

        if(Math.abs(acceleration.accelZ) < 0.1 || loopCycles < 1000) {
            correctedaZ = 0;
        }
        else
        {
            correctedaZ =(float) acceleration.accelZ;
        }
        // Set up our dashboard computations
        composeDashboard();

        // Wait until we're told to go
        waitForStart();

        // Loop and update the dashboard
        while (opModeIsActive())
        {
            if(motorFrontLeft.getPower() == 0)
            {
                prevXvel = 0;
                prevYvel = 0;
                prevZvel = 0;
            }
            if(updateGamepads())
            {
                motorFrontLeft.setPower(MotorControl());
                motorFrontRight.setPower(-MotorControl());
                motorRearLeft.setPower(MotorControl());
                motorRearRight.setPower(-MotorControl());
            }
            telemetry.update();
            idle();
        }
    }

    double MotorControl()
    {
        double GRAPH_X_VALUE = gamepad1.left_stick_y;
        if(GRAPH_X_VALUE > 0){
            double exponentTerm = 2*Math.exp(GRAPH_K_VALUE*((2*GRAPH_X_VALUE)-1));
            return (GRAPH_H_VALUE/(2+exponentTerm))-(GRAPH_H_VALUE/4)+(1/2);
        }
        else{
            double exponentTerm = 2*Math.exp(GRAPH_K_VALUE*((2*GRAPH_X_VALUE)+1));
            return (GRAPH_H_VALUE/(2+exponentTerm))-(GRAPH_H_VALUE/4)-(1/2);
        }

    }

    //----------------------------------------------------------------------------------------------
    // dashboard configuration
    //----------------------------------------------------------------------------------------------
    void composeDashboard()
    {
        // The default dashboard update rate is a little to slow for us, so we update faster
        telemetry.setUpdateIntervalMs(200);

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new IAction() { @Override public void doAction()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles     = imu.getAngularOrientation();
            position   = imu.getPosition();
            acceleration = imu.getLinearAcceleration();

            // The rest of this is pretty cheap to acquire, but we may as well do it
            // all while we're gathering the above.
            loopCycles = getLoopCount();
            i2cCycles  = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
            ms         = elapsed.time() * 1000.0;
            prevXvel += timeInterval*correctedaX;
            prevYvel += timeInterval*correctedaY;
            prevZvel += timeInterval*correctedaZ;
            totalXdisplacement += prevXvel*timeInterval + (correctedaX*Math.pow(timeInterval,2))/2;
            totalYdisplacement += prevYvel*timeInterval + (correctedaY*Math.pow(timeInterval,2))/2;
            totalZdisplacement += prevZvel*timeInterval + (correctedaZ*Math.pow(timeInterval,2))/2;
            if(Math.abs(acceleration.accelX) < 0.1 || (loopCycles < 1000)){
                correctedaX = 0;
            }
            else{
                correctedaX =(float) acceleration.accelX;
            }

            if(Math.abs(acceleration.accelY) < 0.1 || loopCycles < 1000)
            {
                correctedaY = 0;
            }
            else
            {
                correctedaY =(float) acceleration.accelY;
            }

            if(Math.abs(acceleration.accelZ) < 0.1 || loopCycles < 1000) {
                correctedaZ = 0;
            }
            else
            {
                correctedaZ =(float) acceleration.accelZ;
            }
        }
        });
        telemetry.addLine(
                telemetry.item("loop count: ", new IFunc<Object>() {
                    public Object value() {
                        return loopCycles;
                    }
                }),
                telemetry.item("i2c cycle count: ", new IFunc<Object>() {
                    public Object value() {
                        return i2cCycles;
                    }
                }));

        telemetry.addLine(
                telemetry.item("loop rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatRate(ms / loopCycles);
                    }
                }),
                telemetry.item("i2c cycle rate: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatRate(ms / i2cCycles);
                    }
                }));

        telemetry.addLine(
                telemetry.item("status: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeStatus(imu.getSystemStatus());
                    }
                }),
                telemetry.item("calib: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return decodeCalibration(imu.read8(IBNO055IMU.REGISTER.CALIB_STAT));
                    }
                }));

        telemetry.addLine(
                telemetry.item("heading: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.heading);
                    }
                }),
                telemetry.item("roll: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.roll);
                    }
                }),
                telemetry.item("pitch: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAngle(angles.pitch);
                    }
                }));

        telemetry.addLine(
                telemetry.item("ax: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAcceleration(correctedaX);
                    }
                }),
                telemetry.item("ay: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAcceleration(correctedaY);
                    }
                }),
                telemetry.item("az: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatAcceleration(correctedaZ);
                    }
                }));

        telemetry.addLine(
                telemetry.item("x: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(totalXdisplacement);
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(totalYdisplacement);
                    }
                }),
                telemetry.item("z: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(totalZdisplacement);
                    }
                }));
        telemetry.addLine(
                telemetry.item("Motor speed: ",  new IFunc<Object>() {
                            public Object value() {
                                return motorFrontLeft.getPower();
                            }
                        }));

    }

    String formatAngle(double angle)
    {
        return parameters.angleunit==IBNO055IMU.ANGLEUNIT.DEGREES ? formatDegrees(angle) : formatRadians(angle);
    }
    String formatRadians(double radians)
    {
        return formatDegrees(degreesFromRadians(radians));
    }
    String formatDegrees(double degrees)
    {
        return String.format("%.1f", normalizeDegrees(degrees));
    }
    String formatRate(double cyclesPerSecond)
    {
        return String.format("%.2f", cyclesPerSecond);
    }
    String formatPosition(double coordinate)
    {
        String unit = parameters.accelunit== IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }
    String formatAcceleration(double coordinate)
    {
        String unit = parameters.accelunit== IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC
                ? "m/s^2" : "??";
        return String.format("%.2f%s", coordinate, unit);
    }

    //----------------------------------------------------------------------------------------------
    // Utility
    //----------------------------------------------------------------------------------------------

    /** Normalize the angle into the range [-180,180) */
    double normalizeDegrees(double degrees)
    {
        while (degrees >= 180.0) degrees -= 360.0;
        while (degrees < -180.0) degrees += 360.0;
        return degrees;
    }
    double degreesFromRadians(double radians)
    {
        return radians * 180.0 / Math.PI;
    }

    /** Turn a system status into something that's reasonable to show in telemetry */
    String decodeStatus(int status)
    {
        switch (status)
        {
            case 0: return "idle";
            case 1: return "syserr";
            case 2: return "periph";
            case 3: return "sysinit";
            case 4: return "selftest";
            case 5: return "fusion";
            case 6: return "running";
        }
        return "unk";
    }

    /** Turn a calibration code into something that is reasonable to show in telemetry */
    String decodeCalibration(int status)
    {
        StringBuilder result = new StringBuilder();

        result.append(String.format("s%d", (status >> 2) & 0x03));  // SYS calibration status
        result.append(" ");
        result.append(String.format("g%d", (status >> 2) & 0x03));  // GYR calibration status
        result.append(" ");
        result.append(String.format("a%d", (status >> 2) & 0x03));  // ACC calibration status
        result.append(" ");
        result.append(String.format("m%d", (status >> 0) & 0x03));  // MAG calibration status

        return result.toString();
    }
}
