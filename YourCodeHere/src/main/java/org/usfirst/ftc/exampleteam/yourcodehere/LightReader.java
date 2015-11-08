package com.qualcomm.ftcrobotcontroller.opmodes;

        import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;

        import org.swerverobotics.library.interfaces.Autonomous;

/**
 *   Simple program to read and display on DS the reflected light value from the ODS
 *   in order to determine threshold value for line detection.
 *   note:
 *   requires Swerve Robotics libraries, since it used @Autonomous registration
 */


@Autonomous(name="MyLightReader")//add to registration file
public class LightReader extends OpMode {

    //Declare component variables

        OpticalDistanceSensor distanceSensor;

    @Override
    public void init() {

        //Initialize sensor to read robot configuration for "ods"
        distanceSensor = hardwareMap.opticalDistanceSensor.get("ods");
       }

    @Override
    public void loop() {
        double value = distanceSensor.getLightDetectedRaw();

        telemetry.addData("ReflectedLight = ", value);

    }
}