package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.*;
import org.swerverobotics.library.*;
import org.swerverobotics.library.interfaces.*;

/**
 * A skeletal example of a first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="MyBasicTeleOp") //name to appear in Driver Station OpMode selection
//@Disabled  //if you un-comment this, it will keep from showing on DriverStation

public class BasicTeleOp extends SynchronousOpMode //Special note: this class name must match file name
{
    // Declare variable for all components to be used. Note initial values set to null. */
    //motors
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorArm = null;
    //servo
    Servo servoHand = null;
    //default arm position variable
    double CLOSED = 0.2;
    double OPEN = 0.8;

    @Override public void main() throws InterruptedException
        {
        /* Initialize all hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
            this.motorLeft = this.hardwareMap.dcMotor.get("motorL");
            this.motorRight = this.hardwareMap.dcMotor.get("motorR");
            this.motorArm = this.hardwareMap.dcMotor.get("Arm");

            //set motor channel to run without encoders
            motorLeft.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            motorRight.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            motorArm.setChannelMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
            //reverse Left motor
            motorLeft.setDirection(DcMotor.Direction.REVERSE);

            this.servoHand = this.hardwareMap.servo.get("Hand");
            //Preset servoHand position
            servoHand.setPosition(OPEN);

        // Wait for the game to start
        waitForStart();

        // telOp Code below...
        while (opModeIsActive())//loop to run while play is active. Until stop button is pressed.
            {
            if (updateGamepads()) //method to read gamepads
                {
                // tank drive
                motorLeft.setPower(gamepad1.left_stick_y);
                motorRight.setPower(gamepad1.right_stick_y);

                // Arm Control- Uses dual buttons to control motor direction
                if(gamepad1.right_bumper)
                {
                    motorArm.setPower(-gamepad1.right_trigger); // if both Bumper + Trigger, then negative power, runs arm down
                }
                else 
                {
                    motorArm.setPower(gamepad1.right_trigger);  // else trigger positive value, runs arm up
                }
                    
                //servo commands
                if(gamepad1.a)
                {
                    servoHand.setPosition(OPEN); //button 'a' will open
                }
                else if (gamepad1.b)
                {
                    servoHand.setPosition(CLOSED);//button 'b' will close
                }

                }//if updateGamepads

            telemetry.update(); //send telemetry to driver station
            idle();
            }//while opModeActive

    }//main
}//MyFirstOpMode
