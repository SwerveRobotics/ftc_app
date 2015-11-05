package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="HHG8271TeleOpMode")
//@Disabled  //if you un-comment this, it will keep from showing on DriverStation

public class HHG8271TeleOp extends SynchronousOpMode
{
    // Declare here any fields you might find useful. */
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm = null;
    DcMotor motorExtend = null;

    @Override public void main() throws InterruptedException
    {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");
        this.motorArm = this.hardwareMap. dcMotor.get("motorArm");
        this.motorExtend = this.hardwareMap. dcMotor.get("motorExtend");

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive())
        {
            if (updateGamepads())
            {
                //Tank drive motors
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);

                //Run Arm and extend drive.  Uses dual button to run negative power

                if (gamepad1.left_bumper)                       //if Left bumper+trigger, then Negative arm motor power
                {
                    motorArm.setPower(-gamepad1.left_trigger);
                }
                else                                            //else positive arm motor power
                {
                    motorArm.setPower(gamepad1.left_trigger);
                }

                if (gamepad1.right_bumper)                      //same type of controls for extend motor, with right_bumper
                {
                    motorExtend.setPower(-gamepad1.right_trigger);
                }
                else
                {
                    motorExtend.setPower(gamepad1.right_trigger);
                }
            }

            telemetry.update();
            idle();
        }//opMOdeActive
    }//Main
}//HHG8271TeleOp