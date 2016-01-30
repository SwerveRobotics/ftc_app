package org.usfirst.ftc.exampleteam.yourcodehere;

import android.os.Environment;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.IFunc;

import java.io.File;
import java.io.PrintWriter;

/**
 * Tests batteries by slowly draining them via
 * a motor (a resistor wired to a motor port)
 * and periodically measuring the voltage
 */
@Autonomous(name = "Battery Test")
public class BatteryTest extends SynchronousOpMode
{
    DcMotor motor = null;
    VoltageSensor voltageSensor;

    int minimumSafeVoltage = 11;
    ElapsedTime eTime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    // How often logs measurements should be taken, in seconds
    int period = 5;

    // The number of log readings taken thus far
    int readings = 1;

    // The total time to run the test, in minutes
    int testTime = 1;

    String FILENAME = "SwerveBatteryLogger.txt";
    PrintWriter outputFile;

    boolean keepRunning = true;

    @Override
    public void main() throws InterruptedException
    {
        motor = hardwareMap.dcMotor.get("motor");
        voltageSensor = hardwareMap.voltageSensor.get("Motor Controller 1");
        composeDashboard();
        openPublicFileForWriting(FILENAME);

        waitForStart();

        eTime.reset();
        telemetry.log.add("Start Voltage: " + formatNumber(voltageSensor.getVoltage()));

        //write initial unloaded voltage to the file
        writeDataToPublicFile(voltageSensor.getVoltage());

        motor.setPower(1.0);

        while (this.opModeIsActive() && keepRunning)
        {
            /*
            // Break if elapsed time has exceeded test time
            if(eTime.time() >= testTime * 60)
            {
                telemetry.log.add("[STOPPED] Test time exceeded");
                break;
            }*/

            // Break if battery voltage drops below minimum safe value
            if(voltageSensor.getVoltage() < minimumSafeVoltage)
            {
                telemetry.log.add("[STOPPED] Battery voltage below minimum safe value");
                keepRunning = false;
            }

            // Checks to see if another period has passed
            if(eTime.time() - (readings * period) >= 0)
            {
                //telemetry.log.add("Voltage at " + readings * period + " seconds: " + formatNumber(voltageSensor.getVoltage()));
                writeDataToPublicFile(voltageSensor.getVoltage());
                readings += 1;
            }

            telemetry.update();
            idle();
        }

        motor.setPower(0.0);
        telemetry.log.add("End Voltage: " + formatNumber(voltageSensor.getVoltage()));
        closePublicFile();
    }

    void composeDashboard()
    {
        telemetry.addLine
                (
                        this.telemetry.item("Current Voltage: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(voltageSensor.getVoltage());
                            }
                        })
                );
        telemetry.addLine
                (
                        this.telemetry.item("Readings: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(readings);
                            }
                        })
                );

        telemetry.addLine
                (
                        this.telemetry.item("Elapsed Time: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(eTime.time());
                            }
                        })
                );

        /*
         * 3 extra lines are added, because there are messages
         * getting logged other than the periodical voltage reading
         */
        //telemetry.log.setCapacity(testTime * 60 / period + 3);
    }

    public String formatNumber(double number)
    {
        return String.format("%.2f", number);
    }

    public void openPublicFileForWriting(String filename)
    {
        String fullpath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + filename;

        try
        {
            File file = new File(fullpath);
            if (!file.exists())
            {
                file.createNewFile();
            }
            outputFile = new PrintWriter(file);
        }
        catch (Exception e)
        {
            telemetry.log.add("Exception opening file: " + e.toString());
        }
    }

    public void writeDataToPublicFile(String message)
    {
        if (outputFile!=null)
        {
            try
            {
                outputFile.println(message);
                outputFile.flush();
            }
            catch (Exception e)
            {
                telemetry.log.add("Exception writing to file: " + e.toString());
            }
        }
    }

    public void writeDataToPublicFile(double voltage)
    {
        String s = formatNumber(eTime.time()) + "," + formatNumber(voltage) + "\r\n";
        writeDataToPublicFile(s);
    }

    public void closePublicFile()
    {
        if (outputFile != null)
        {
            try
            {
                outputFile.flush();
                outputFile.close();
            }
            catch (Exception e)
            {
                telemetry.log.add("Exception closing file: " + e.toString());
            }
        }
    }
}


