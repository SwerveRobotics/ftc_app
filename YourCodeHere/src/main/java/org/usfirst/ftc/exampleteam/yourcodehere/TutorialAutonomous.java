package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Autonomous program made for the "Basic Autonomous" video
 */
@Autonomous(name="Tutorial Autonomous")
@Disabled
public class TutorialAutonomous extends SynchronousOpMode
{
    // Declare motors
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
        armServo = hardwareMap.servo.get("armServo");

        RaiseArm();

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!

        DriveForwardTime(DRIVE_POWER, 4000);
        TurnLeftTime(DRIVE_POWER, 500);
        DriveForwardTime(DRIVE_POWER, 4000);
        TurnRightTime(DRIVE_POWER, 500);
        DriveForwardTime(DRIVE_POWER, 4000);
        StopDriving();
        LowerArm();
    }

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

    public void TurnLeft(double power)
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
    }

    public void TurnLeftTime(double power, long time) throws InterruptedException
    {
        TurnLeft(power);
        Thread.sleep(time);
    }

    public void TurnRight(double power)
    {
        TurnLeft(-power);
    }

    public void TurnRightTime(double power, long time) throws InterruptedException
    {
        TurnRight(power);
        Thread.sleep(time);
    }

    double ARM_HIGH_POS = 0.8;
    double ARM_LOW_POS = 0.2;

    public void RaiseArm()
    {
        armServo.setPosition(ARM_HIGH_POS);
    }

    public void LowerArm()
    {
        armServo.setPosition(ARM_LOW_POS);
    }
}
