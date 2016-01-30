package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cDevice;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Disabled;
import org.swerverobotics.library.interfaces.IFunc;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@TeleOp(name="MultiplexerExample")
@Disabled
public class MultiplexerExample extends SynchronousOpMode
    {
        I2cDevice multiplexer = null;
        ColorSensor colorSensor1 = null;
        ColorSensor colorSensor2 = null;

    @Override public void main() throws InterruptedException
        {
            multiplexer = hardwareMap.i2cDevice.get("multiplexer");
            setMultiplexerPort((byte)0x01);
            colorSensor1 = hardwareMap.colorSensor.get("colorsensor1");

        // Wait for the game to start
        waitForStart();

        // Go go gadget robot!
        while (opModeIsActive())
            {
            if (updateGamepads())
                {
                    if(gamepad1.a)
                    {
                        setMultiplexerPort((byte)0x01);
                        telemetry.addLine(
                                telemetry.item("Color: ", new IFunc<Object>() {
                                    @Override
                                    public Object value() {
                                        return "r: " + colorSensor1.red() + "g: " + colorSensor1.green() + "b: " + colorSensor1.blue();
                                    }
                                }));
                    }
                    if (gamepad1.b)
                    {
                        setMultiplexerPort((byte)0x04);
                        telemetry.addLine(
                                telemetry.item("Color: ", new IFunc<Object>() {
                                    @Override
                                    public Object value() {
                                        return "r: " + colorSensor2.red() + "g: " + colorSensor2.green() + "b: " + colorSensor2.blue();
                                    }
                                }));

                    }
                }

            telemetry.update();
            idle();
            }
        }

        public void setMultiplexerPort(byte Port)
        {
            byte[] portSwitch = new byte[1];

            portSwitch[0] = Port; //port 0

            multiplexer.enableI2cWriteMode(0x70, 0, 1);
            multiplexer.copyBufferIntoWriteBuffer(portSwitch);
            multiplexer.writeI2cCacheToController();

        }
    }
