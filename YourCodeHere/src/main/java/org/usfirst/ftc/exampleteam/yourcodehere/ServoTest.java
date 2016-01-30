package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.Servo;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="Servo Test")
@Disabled
public class ServoTest extends SynchronousOpMode
{
    /* Declare here any fields you might find useful. */
    // DcMotor motorLeft = null;
    // DcMotor motorRight = null;
    Servo servoLeftZipline = null;
    Servo servoRightZipline = null;
    Servo servoTapeMeasureElevation = null;
    Servo servoCollectorHinge = null;

    @Override public void main() throws InterruptedException
    {
        /* Initialize our hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names you assigned during the robot configuration
         * step you did in the FTC Robot Controller app on the phone.
         */
        // this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        // this.motorRight = this.hardwareMap.dcMotor.get("motorRight");


        servoLeftZipline = hardwareMap.servo.get("servoLeftZipline");
        servoRightZipline = hardwareMap.servo.get("servoRightZipline");
        servoTapeMeasureElevation = hardwareMap.servo.get("servoTapeMeasureElevation");
        servoCollectorHinge = hardwareMap.servo.get("servoCollectorHinge");

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive())
        {
            if (updateGamepads())
            {
                if(gamepad1.a)
                {
                    servoLeftZipline.setPosition(servoLeftZipline.getPosition() + 0.1);
                    servoRightZipline.setPosition(servoRightZipline.getPosition() + 0.1);
                    servoTapeMeasureElevation.setPosition(servoTapeMeasureElevation.getPosition() +0.1);
                    servoCollectorHinge.setPosition(servoCollectorHinge.getPosition() + 0.1);
                }
                else if(gamepad1.b)
                {
                    servoLeftZipline.setPosition(servoLeftZipline.getPosition() - 0.1);
                    servoRightZipline.setPosition(servoRightZipline.getPosition() - 0.1);
                    servoTapeMeasureElevation.setPosition(servoTapeMeasureElevation.getPosition() - 0.1);
                    servoCollectorHinge.setPosition(servoCollectorHinge.getPosition() - 0.1);
                }
            }

            telemetry.update();
            idle();
        }
    }
}
