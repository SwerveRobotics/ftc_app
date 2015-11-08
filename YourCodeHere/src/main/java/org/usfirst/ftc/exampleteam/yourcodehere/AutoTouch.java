package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Autonomous program made from the "Basic Autonomous" video by SwerveRobotics
 * Uses power/time based motor movement to achieve desired distance.
 * Robot configuration include: tank drive motors, 1 servo for arm positioning
 *
 * with these additional features:
 * Changed wait() methods to Thread.sleep() Note: the wait() method originally shown in video did not function
 * Added a StopDrivingTime() to create a pause in the program
 */
@Autonomous(name="MyAutoTouch") //name to appear in Driver Station OpMode selection
//@Disabled  //if you un-comment this, it will keep from showing on DriverStation

public class AutoTouch extends SynchronousOpMode
{
    /* Declare variable for all components to be used. Note initial values set to null. */

    //Declare Motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    // Declare servos
    Servo armServo = null;

    //Declare Sensors
    TouchSensor touch = null;

    @Override public void main() throws InterruptedException
    {
        // Initialize motors
        motorLeft = hardwareMap.dcMotor.get("motorL");
        motorRight = hardwareMap.dcMotor.get("motorR");

       // motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);  //setChannelMode has been deprecated
       // motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos
        armServo = hardwareMap.servo.get("servoHandL");

        // Initialize sensors
        touch = hardwareMap.touchSensor.get("touch");

        //  Set arm position for start
        LowerArm();

        // Wait for the game to start
        waitForStart();

        /************************
        * Autonomous Code Below://
        *************************/

        while (!touch.isPressed())  //drive forward until touch is pressed
        {
            DriveForward(DRIVE_POWER);
        }

        StopDrivingTime(2000);  // pause for 2 sec, then proceed with next set of code

        RaiseArm();
        DriveForwardTime(DRIVE_POWER, 4000);
        StopDriving();

    }//Main

// Below: Additional Methods to clean up Main code...
// added time to both turn methods, just like for DriveForwardTime

    double DRIVE_POWER = 1.0;

    public void DriveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void DriveForwardTime(double power, long time) throws InterruptedException
    {
        DriveForward(power);
        Thread.sleep(time);
    }

    public void StopDriving()
    {
        DriveForward(0);
    }

    public void StopDrivingTime(long time) throws InterruptedException
    {
        DriveForwardTime(0, time);
    }

    public void TurnLeft(double power, long time) throws InterruptedException
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        Thread.sleep(time);
    }

    public void TurnRight(double power, long time) throws InterruptedException
    {
        TurnLeft(-power, time);
    }

    public void RaiseArm()
    {
        armServo.setPosition(.8);
    }

    public void LowerArm()
    {
        armServo.setPosition(.2);
    }

    /* this Delay method is no longer needed for above code. Will leave in for possible future use */
    private void Delay(long msToDelay)
    {
        // Protect from silly programming errors
        if (msToDelay > 0)
        {
            try {
                Thread.sleep(msToDelay);
            } catch (InterruptedException ignored) {
            }
        }
    }

}//<MyTutorialAuto