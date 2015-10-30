package org.swerverobotics.library.examples;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.Position;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * An example of a synchronous opmode that implements a simple drive-a-bot.
 */
@TeleOp(name="6220Auto", group="Swerve Examples")
public class Synch6220Autonomous extends SynchronousOpMode
{
    // All hardware variables can only be initialized inside the main() function,
    // not here at their member variable declarations.
    DcMotor MotorRightBack = null;
    DcMotor MotorLeftBack = null;
    DcMotor Motor3 = null;
    DcMotor MotorLeftFront = null;
    DcMotor MotorRightFront = null;
    DcMotor Motor6 = null;

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
        this.MotorRightBack = this.hardwareMap.dcMotor.get("Motor1");
        this.MotorLeftBack = this.hardwareMap.dcMotor.get("Motor2");
        this.Motor3 = this.hardwareMap.dcMotor.get("Motor3");
        this.MotorLeftFront = this.hardwareMap.dcMotor.get("Motor4");
        this.MotorRightFront = this.hardwareMap.dcMotor.get("Motor5");
        this.Motor6 = this.hardwareMap.dcMotor.get("Motor6");

        // Configure the knobs of the hardware according to how you've wired your
        // robot. Here, we assume that there are no encoders connected to the motors,
        // so we inform the motor objects of that fact.
        this.MotorRightBack.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.MotorLeftBack.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.Motor3.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.MotorLeftFront.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.MotorRightFront.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        this.Motor6.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // One of the two motors (here, the left) should be set to reversed direction
        // so that it can take the same power level values as the other motor.
        this.MotorRightBack.setDirection(DcMotor.Direction.REVERSE);
        //this.Motor3.setDirection(DcMotor.Direction.REVERSE);
        //this.MotorLeftFront.setDirection(DcMotor.Direction.REVERSE);

        // We are expecting the IMU to be attached to an I2C port on  a core device interface
        // module and named "imu". Retrieve that raw I2cDevice and then wrap it in an object that
        // semantically understands this particular kind of sensor.
        parameters.angleunit      = IBNO055IMU.ANGLEUNIT.DEGREES;
        parameters.accelunit      = IBNO055IMU.ACCELUNIT.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag     = "BNO055";
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);

        // Enable reporting of position using the naive integrator
        //imu.startAccelerationIntegration(new Position(), new Velocity());


        // Configure the dashboard however we want it
        this.configureDashboard();

        // Wait until we've been given the ok to go
        this.waitForStart();

        //autonomous code starts here

        drive (1);
        Thread.sleep(1000);
        halt();
        turn (45, .5);
        halt ();

    }

    void halt ()
    {
       drive(0);
    }

    void drive (double power)
    {
        // Tell the motors
        this.MotorRightBack.setPower(power);
        this.MotorLeftFront.setPower(0);
        this.MotorRightFront.setPower(0);
        //this.Motor6.setPower(power1);
        this.MotorLeftBack.setPower(power);
        //this.Motor3.setPower(power2);
    }

    void drive (double leftpower, double rightpower)
    {
        // Tell the motors
        this.MotorRightBack.setPower(leftpower);
        this.MotorLeftFront.setPower(0);
        this.MotorRightFront.setPower(0);
        //this.Motor6.setPower(power1);
        this.MotorLeftBack.setPower(rightpower);
        //this.Motor3.setPower(power2);
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

            drive(power, -power);

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

            drive(-power, power);

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
        halt ();
    }

    void configureDashboard()
    {
        // Configure the dashboard. Here, it will have one line, which will contain three items
        this.telemetry.addLine
                (
                        this.telemetry.item("left:", new IFunc<Object>()
                        {
                            @Override public Object value()
                            {
                                return format(MotorRightBack.getPower());
                            }
                        }),
                        this.telemetry.item("right: ", new IFunc<Object>()
                        {
                            @Override public Object value()
                            {
                                return format(MotorRightBack.getPower());
                            }
                        }),
                        this.telemetry.item("mode: ", new IFunc<Object>()
                        {
                            @Override public Object value()
                            {
                                return MotorRightBack.getChannelMode();
                            }
                        })
                );
    }

    // Handy functions for formatting data for the dashboard
    String format(double d)
    {
        return String.format("%.1f", d);
    }



}
