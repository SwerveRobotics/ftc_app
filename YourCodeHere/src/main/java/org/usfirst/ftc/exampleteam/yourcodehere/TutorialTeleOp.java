package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A basic TeleOp program made for the "Basic TeleOp" video
 */
@TeleOp(name="Tutorial TeleOp")
@Disabled
public class TutorialTeleOp extends SynchronousOpMode
{
    // Declare motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;

    // Declare servos
    Servo servoArm = null;

    // Default servo positions
    double ARM_MIN = 0.2;
    double ARM_MAX = 0.8;

    @Override public void main() throws InterruptedException
    {
        // Initialize motors
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");

        // Initialize servos
        servoArm = hardwareMap.servo.get("servoArm");

        // Set motor channel modes
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // Reverse left drive motors so we don't spin when applying full power
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        // Initialize servo position to stay inside the 18 inch box
        servoArm.setPosition(ARM_MAX);

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive())
        {
            if (updateGamepads())
            {
                // Tank drive
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);

                // Move the arm
                if(gamepad2.a)
                {
                    servoArm.setPosition(ARM_MIN);
                }
                else if(gamepad2.b)
                {
                    servoArm.setPosition(ARM_MAX);
                }
            }

            telemetry.update();
            idle();
        }
    }
}
