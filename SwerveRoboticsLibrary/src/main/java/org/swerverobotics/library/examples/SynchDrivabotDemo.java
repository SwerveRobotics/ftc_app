package org.swerverobotics.library.examples;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.TeleOp;
import org.swerverobotics.library.interfaces.Velocity;

/**
 * An example of a synchronous opmode that implements a simple drive-a-bot. 
 */
@TeleOp(name="Drivabot", group="Swerve Examples")
public class SynchDrivabotDemo extends SynchronousOpMode
{
    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    //LED led = null;

    // Our sensors, motors, and other devices go here, along with other long term state
    IBNO055IMU imu;
    ElapsedTime elapsed    = new ElapsedTime();
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    // Here we have state we use for updating the dashboard. The first of these is important
    // to read only once per update, as its acquisition is expensive. The remainder, though,
    // could probably be read once per item, at only a small loss in display accuracy.
    EulerAngles angles;
    Position position;
    int                     loopCycles;
    int                     i2cCycles;
    double                  ms;

    @Override protected void main() throws InterruptedException
    {
        // Initialize our hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names you assigned during the robot configuration
        // step you did in the FTC Robot Controller app on the phone.
        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");
        //this.led = this.hardwareMap.led.get("led");

        // Configure the knobs of the hardware according to how you've wired your
        // robot. Here, we assume that there are no encoders connected to the motors,
        // so we inform the motor objects of that fact.
        this.motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // One of the two motors (here, the left) should be set to reversed direction
        // so that it can take the same power level values as the other motor.
        this.motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // We are expecting the IMU to be attached to an I2C port on  a core device interface
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        imu.startAccelerationIntegration(new Position(), new Velocity());

        // Configure the dashboard however we want it
        this.configureDashboard();

        // Wait until we've been given the ok to go
        this.waitForStart();

        while (!this.opModeIsActive())
        {
            //do nothing until the opmode is active
        }

        angles     = imu.getAngularOrientation();

        // Enter a loop processing all the input we receive
        while (this.opModeIsActive())
        {
            if (this.updateGamepads())
            {
                // There is (likely) new gamepad input available.
                // Do something with that! Here, we just drive.
                this.doManualDrivingControl(this.gamepad1);
               // if (gamepad1.a) led.enable(true);
               // if (gamepad1.b) led.enable(false);

                if (gamepad1.b) {

                    this.moveForward(300, 0.2);
                    this.turn(-30, 0.2);
                    this.moveForward(4000, 0.2);
                    this.turn(-25, 0.2);
                    this.moveForward(400, 0.2);

                }

                if (gamepad1.x) {

                    //this.TurnLeft(45);
                    this.turn(-45, 0.4);

                }


                if (gamepad1.y)
                {
                    //this.TurnRight(45);
                    this.turn(45, 0.4);

                }

                if (gamepad1.left_bumper)
                {
                    //this.TurnRight(45);
                    this.turn(-90, 0.4);
                }

                if (gamepad1.right_bumper)
                {
                    //this.TurnRight(45);
                    this.turn(90, 0.4);
                }

                if (gamepad1.dpad_left)
                {
                    //this.TurnRight(45);
                    this.turn(-270, 0.4);
                }

                if (gamepad1.dpad_right)
                {
                    //this.TurnRight(45);
                    this.turn(270, 0.4);
                }




            }

            // Emit telemetry with the freshest possible values
            this.telemetry.update();

            // Let the rest of the system run until there's a stimulus from the robot controller runtime.
            this.idle();
        }
    }


    double normalizeAngle180(double angle){
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle +=360;
        return angle;
    }


    double normalizeAngle360(double angle)
    {
        while (angle >= 360) angle -= 360;
        while (angle < 0) angle += 360;
        return angle;
    }

    double getCurrentHeading()
    {
        angles     = imu.getAngularOrientation();

        // Emit telemetry with the freshest possible values
        this.telemetry.update();

        return angles.heading;
    }

    void moveForward(long timeInMillis, double power) throws InterruptedException
    {
        motorLeft.setPower(-power);
        motorRight.setPower(-power);

        Thread.sleep(timeInMillis);

        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    void turn(double delta, double power)
    {
        //richTextBoxResult.Text = "";

        int full270s = (int)Math.abs(delta / 270);
        if (delta >=0)
        {
            for (int i = 0; i < full270s; i++) TurnUpTo270Degrees(270, power);
        }
        else
        {
            for (int i = 0; i < full270s; i++) TurnUpTo270Degrees(-270, power);
        }

        double remainder =  delta % 270;
        TurnUpTo270Degrees(remainder, power);
    }

    void TurnUpTo270Degrees(double delta, double power)
    {
        //ASSERT(delta<=270)
        double start_heading = getCurrentHeading();
        double cur = start_heading;
        double dest = cur + delta;

        //normalize dest to between 0..360
        dest = normalizeAngle360(dest);

        boolean goLeft = (delta < 0 ? true : false);

        //int watchdog = 0;

        double GUARD = 40; //protection region against jolts that make the robot jump past the end point or before the start point
        double offset = 0; //amount to offset angles to ensure we don't cross over the 359.99..0 discontinuity during the turn

        /*
        String result = "";

        result += "start: " + start_heading.ToString() + "\n";
        result += "cur: " + cur.ToString() + "\n";
        result += "dest: " + dest.ToString() + "\n";
        result += "direction: " + (goLeft ? "left" : "right") + "\n";
        result += "\n\n";
        */

        if (goLeft) //go left: moving in negative direction around compass
        {
            //if the dest is normalized to a value larger than the current heading,
            //it means that the compass will pass 0 (going in the negative direction) on the way to the dest.
            //When that happens, the current heading will jump from 0 to 359.
            //We will offset our angles to ensure that we don't have that problem.
            if (cur < dest)
            {
                offset = (360 - dest) + GUARD; //put dest at GUARD, move cur by the same amount
            }
            //We won't cross 0 in this turn, but we still want to put some space between
            //the start/end angles and the discontinuity at 0/360
            else
            {
                offset = (GUARD - dest); //put dest at GUARD, move cur by the same amount
            }

            //result += "offset: " + offset.ToString() + "\n";

            cur = normalizeAngle360(cur + offset);
            dest = normalizeAngle360(dest + offset);

            motorLeft.setPower(power);
            motorRight.setPower(-power);

            while ((cur > dest))
            {
                //going left = --
                cur = normalizeAngle360(getCurrentHeading() + offset);
                //watchdog++;
            }
        }
        else //go right: moving in positive direction around compass
        {
            //if the dest is normalized to a value less than than the current heading,
            //it means that the compass will pass 0 on the way to the dest.
            //When that happens, the current heading will jump from 359 to 0.
            //We will offset our angles to ensure that we don't have that problem.
            if (cur > dest)
            {
                offset = (360 - cur) + GUARD; //put cur at GUARD, move dest by the same amount
            }
            else
            {
                offset = (GUARD - cur); //put cur at GUARD, move dest by the same amount
            }

            cur = normalizeAngle360(cur + offset);
            dest = normalizeAngle360(dest + offset);

            motorLeft.setPower(-power);
            motorRight.setPower(power);

            //now compass 0 doesn't fall between the current heading and the destination,
            //so we can use a < comparison to wait for our target
            while ((cur < dest)) {
                //result += cur.ToString() + "\n";
                //going left = --
                cur = normalizeAngle360(getCurrentHeading() + offset);
                //watchdog++;
            }
        }

        //result += "dest reached: " + cur.ToString() + "\n";
        motorLeft.setPower(0);
        motorRight.setPower(0);
    }

    /* turn a specific angle (a delta from the current heading) at a given percentage of power
       Positive deltas result in Right turns.
       In other words, turn(40, .4) will turn 40 degrees to the right at 40% power.
     */
    /*
    void turn(double delta, double power)
    {
        double start_heading = getCurrentHeading();
        double cur = start_heading;
        double dest = cur + delta;

        //normalize dest to between 0..360
        // because when we add an angle to start_heading, which is 0..360, we might go out of range of 0..360
        dest = normalizeAngle360(dest);

        boolean goLeft = (delta < 0 ? true : false);

        if (goLeft) //go left: moving in negative direction around compass
        {
            motorLeft.setPower(power);
            motorRight.setPower(-power);

            //if the dest is normalized to a value larger than the current heading,
            //it means that the compass will pass 0 on the way to the dest.
            //When that happens, the current heading will jump from 0 to 359
            //so we need to get past that point
            if (cur < dest)
            {
                //get across 0 boundary

                while (cur <= start_heading)
                {
                    cur = getCurrentHeading();
                }
            }

            //no 0 boundary

            double wrap_point = cur; //watch for wrapping in boundary conditions

            //now compass 0 doesn't fall between the current heading and the destination,
            //so we can use a > comparison to wait for our target
            //but we still need to watch for wrapping around the compass in case
            //the dest is close to 0 and the compass wraps before we think we're done
            while ((cur > dest) && (wrap_point>=cur))
            {
                cur = getCurrentHeading();
            }
        }
        else //go right: moving in positive direction around compass
        {
            motorLeft.setPower(-power);
            motorRight.setPower(power);

            //if the dest is normalized to a value less than than the current heading,
            //it means that the compass will pass 0 on the way to the dest.
            //When that happens, the current heading will jump from 359 to 0
            //so we need to get past that point
            if (cur > dest)
            {
                while (cur >= start_heading)
                {
                    cur = getCurrentHeading();
                }
            }

            double wrap_point = cur; //watch for wrapping in boundary conditions

            //now compass 0 doesn't fall between the current heading and the destination,
            //so we can use a < comparison to wait for our target
            //but we still need to watch for wrapping around the compass in case
            //the dest is close to 0 and the compass wraps before we think we're done
            while ((cur < dest) && (wrap_point <= cur))
            {
                cur = getCurrentHeading();
            }
        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }
    */
/*
    void TurnRight(double targetAngle) throws InterruptedException
    {
        angles     = imu.getAngularOrientation();

        double currentheading = normalizeAngle180(angles.heading);
        double targetheading = normalizeAngle180(currentheading - targetAngle);

        motorLeft.setPower(-.4);
        motorRight.setPower(.4);

        while (currentheading >= targetheading){

            // Emit telemetry with the freshest possible values
            //this.telemetry.update();

            angles     = imu.getAngularOrientation();
            currentheading = normalizeAngle180(angles.heading);

            // Let the rest of the system run until there's a stimulus from the robot controller runtime.
            this.idle();


        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

    void TurnLeft(double targetAngle) throws InterruptedException
    {
        angles     = imu.getAngularOrientation();

        double currentheading = normalizeAngle180(angles.heading);
        double targetheading = normalizeAngle180(targetAngle + currentheading);

        motorLeft.setPower(.4);
        motorRight.setPower(-.4);

        while (currentheading <= targetheading){

            // Emit telemetry with the freshest possible values
            //this.telemetry.update();

            angles     = imu.getAngularOrientation();
            currentheading = normalizeAngle180(angles.heading);
            // Let the rest of the system run until there's a stimulus from the robot controller runtime.
            this.idle();


        }

        motorLeft.setPower(0);
        motorRight.setPower(0);

    }

*/

    /**
     * Implement a simple two-motor driving logic using the left and right
     * right joysticks on the indicated game pad.
     */
    void doManualDrivingControl(Gamepad pad) throws InterruptedException
    {
        // Remember that the gamepad sticks range from -1 to +1, and that the motor
        // power levels range over the same amount
        float ctlPower    =  pad.left_stick_y;
        float ctlSteering =  pad.right_stick_x;

        // We're going to assume that the deadzone processing has been taken care of for us
        // already by the underlying system (that appears to be the intent). Were that not
        // the case, then we would here process ctlPower and ctlSteering to be exactly zero
        // within the deadzone.

        // Map the power and steering to have more oomph at low values (optional)
        ctlPower = this.xformDrivingPowerLevels(ctlPower);
        ctlSteering = this.xformDrivingPowerLevels(ctlSteering);

        // Dampen power to avoid clipping so we can still effectively steer even
        // under heavy throttle.
        //
        // We want
        //      -1 <= ctlPower - ctlSteering <= 1
        //      -1 <= ctlPower + ctlSteering <= 1
        // i.e
        //      ctlSteering -1 <= ctlPower <=  ctlSteering + 1
        //     -ctlSteering -1 <= ctlPower <= -ctlSteering + 1
        ctlPower = Range.clip(ctlPower,  ctlSteering -1,  ctlSteering +1);
        ctlPower = Range.clip(ctlPower, -ctlSteering -1, -ctlSteering +1);

        // Figure out how much power to send to each motor. Be sure
        // not to ask for too much, or the motor will throw an exception.
        float powerLeft  = Range.clip(ctlPower - ctlSteering, -1f, 1f);
        float powerRight = Range.clip(ctlPower + ctlSteering, -1f, 1f);

        // Tell the motors
        this.motorLeft.setPower(powerLeft);
        this.motorRight.setPower(powerRight);
    }

    float xformDrivingPowerLevels(float level)
    // A useful thing to do in some robots is to map the power levels so that
    // low power levels have more power than they otherwise would. This sometimes
    // help give better driveability.
    {
        // We use a log function here as a simple way to transform the levels.
        // You might want to try something different: perhaps construct a
        // manually specified function using a table of values over which
        // you interpolate.
        float zeroToOne = Math.abs(level);
        float oneToTen  = zeroToOne * 9 + 1;
        return (float)(Math.log10(oneToTen) * Math.signum(level));
    }

    void configureDashboard()
    {
        this.telemetry.addLine(
                this.telemetry.item("heading: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return format(angles.heading);
                            }
                        }
                )
        );

        /*

        // Configure the dashboard. Here, it will have one line, which will contain three items
        this.telemetry.addLine
                (
                        this.telemetry.item("left:", new IFunc<Object>()
                        {
                            @Override public Object value()
                            {
                                return format(motorLeft.getPower());
                            }
                        }),
                        this.telemetry.item("right: ", new IFunc<Object>()
                        {
                            @Override public Object value()
                            {
                                return format(motorLeft.getPower());
                            }
                        }),
                        this.telemetry.item("mode: ", new IFunc<Object>()
                        {
                            @Override public Object value()
                            {
                                return motorLeft.getChannelMode();
                            }
                        })
                );
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

            // The rest of this is pretty cheap to acquire, but we may as well do it
            // all while we're gathering the above.
            loopCycles = getLoopCount();
            i2cCycles  = ((II2cDeviceClientUser) imu).getI2cDeviceClient().getI2cCycleCount();
            ms         = elapsed.time() * 1000.0;
        }
        });
        telemetry.addLine(
                telemetry.item("loop count: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return loopCycles;
                    }
                }),
                telemetry.item("i2c cycle count: ", new IFunc<Object>()
                {
                    public Object value()
                    {
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
                telemetry.item("x: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.x);
                    }
                }),
                telemetry.item("y: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.y);
                    }
                }),
                telemetry.item("z: ", new IFunc<Object>()
                {
                    public Object value()
                    {
                        return formatPosition(position.z);
                    }
                }));

                */
    }

    // Handy functions for formatting data for the dashboard
    String format(double d)
    {
        return String.format("%.1f", d);
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

    /** turn a system status into something that's reasonable to show in telemetry */
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

    /** turn a calibration code into something that is reasonable to show in telemetry */
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
