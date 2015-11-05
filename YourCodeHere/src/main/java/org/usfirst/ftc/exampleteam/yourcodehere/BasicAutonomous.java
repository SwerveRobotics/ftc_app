package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.internal.ThreadSafeAnalogInput;

/**
 * Autonomous program made for the "Basic Autonomous" video
 *
 * Changed wait() methods to Thread.sleep()
 *
 * Added a StopDrivingTime() to create a pause in the program
 */
@Autonomous(name="MyBasicAuto") //name to appear in Driver Station OpMode selection
//@Disabled  //if you un-comment this, it will keep from showing on DriverStation

public class BasicAutonomous extends SynchronousOpMode
{
    /* Declare variable for all components to be used. Note initial values set to null. */

    //Declare Motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    // Declare servos
    Servo armServo = null;

    @Override public void main() throws InterruptedException
    {
        // Initialize motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servos
        armServo = hardwareMap.servo.get("servoArm");

        //  Set arm position for start
        LowerArm();

        // Wait for the game to start
        waitForStart();

        // Autonomous Code here://
        DriveForwardTime(DRIVE_POWER, 4000);
        TurnLeft(DRIVE_POWER, 500);
        StopDrivingTime(2000);

        DriveForwardTime(DRIVE_POWER, 4000);
        TurnRight(DRIVE_POWER, 500);
        StopDrivingTime(2000);

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

    public void StopDrivingTime(long time) throws InterruptedException {
        DriveForwardTime(0, time);
    }

    public void TurnLeft(double power, long time) throws InterruptedException {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
        Thread.sleep(time);
    }

    public void TurnRight(double power, long time) throws InterruptedException {
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
    private void Delay(long msToDelay) {
        // Protect from silly programming errors
        if (msToDelay > 0) {
            try {
                Thread.sleep(msToDelay);
            } catch (InterruptedException ignored) {
            }
        }
    }

}//<MyTutorialAuto