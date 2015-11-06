package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.*;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * A skeletal example of a first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="NateTeleOp") //name to appear in Driver Station OpMode selection
@Disabled  //if you un-comment this, it will keep from showing on DriverStation

public class NateOpMode extends SynchronousOpMode //Special note: this class name must match file name
{
    // Declare variable for all components to be used. Note initial values set to null. */
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motor3 = null;

    Servo servoArm = null;
    //default arm position variable
    double ARM_MIN = 0.2;
    double ARM_MAX = 0.8;

    @Override public void main() throws InterruptedException
    {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");
        //legacy motor
        this.motor3 = this.hardwareMap.dcMotor.get("motor3");


        //set motor channel to run without encoders
        motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        //reverse Left motor
        motorLeft.setDirection(DcMotor.Direction.REVERSE);

        this.servoArm = this.hardwareMap.servo.get("servoArm");

        servoArm.setPosition(ARM_MAX);

        // Wait for the game to start
        waitForStart();

        // telOp Code below...zrt33f
        while (opModeIsActive())
        {
            if (updateGamepads())
            {
                // tank drive
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);

                // legacy motor connection
                motor3.setPower(gamepad1.right_trigger);

                //servo commands
                if(gamepad1.a)
                {
                    servoArm.setPosition(ARM_MIN);
                }
                else if (gamepad1.b)
                {
                    servoArm.setPosition(ARM_MAX);
                }


            }//if updateGamepads

            telemetry.update();
            idle();
        }//while opModeActive

    }//main
}//MyFirstOpMode