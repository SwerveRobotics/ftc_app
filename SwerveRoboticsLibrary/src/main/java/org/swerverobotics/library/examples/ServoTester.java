package org.swerverobotics.library.examples;

import com.qualcomm.robotcore.hardware.Servo;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/*
 * This file will test 1 servo based on joystick input
 */
@TeleOp(name="DrywServo", group="Swerve Examples")
@Disabled
public class ServoTester extends SynchronousOpMode
{
    // Declare servo
    Servo servo = null;

    @Override protected void main() throws InterruptedException
    {
        // Initialize servo
        this.servo = this.hardwareMap.servo.get("servo");

        // Configure dashboard
        this.telemetry.addLine
                (
                        this.telemetry.item("Servo:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return servo.getPosition();
                            }
                        })
                );
        
        // Wait until we've been given the ok to go
        this.waitForStart();
        
        // Enter a loop processing all the input we receive
        while (this.opModeIsActive())
        {
            if (this.updateGamepads())
            {
                /*
                 * Move servo corresponding to left joystick values
                 * The range of the joysticks is -1 to 1
                 * The range of the servos is 0 to 1
                 * This adjusts the range of a joystick to be in a servo range
                 */
                servo.setPosition((gamepad1.left_stick_y + 1) / 2);
            }

            // Emit the latest telemetry and wait, letting other things run
            this.telemetry.update();
            this.idle();
        }
    }
}
