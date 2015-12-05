package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LightSensor;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;


/**
 * A skeletal example of a do-nothing first OpMode. Go ahead and change this code
 * to suit your needs, or create sibling OpModes adjacent to this one in the same
 * Java package.
 */
@Autonomous(name="LightSensorCalibration")
public class LightSensorCalibration extends SynchronousOpMode {

    LightSensor LightSensorCalibrator;
    @Override
    public void main() throws InterruptedException {
        LightSensorCalibrator = hardwareMap.lightSensor.get("LightSensorCalibrator");

        waitForStart();

        while (opModeIsActive()) {

            if (updateGamepads()){
                if (gamepad1.a){
                    calibrateBlack();
                }
                if (gamepad1.b){
                    calibrateWhite();
                }

            }
        }

        calibrateBlack();
        calibrateWhite();
        while (true) {
            telemetry.update();
            this.idle();
        }
    }

    public double calibrateBlack() {
        double black;
        black = LightSensorCalibrator.getLightDetected();
        telemetry.log.add("black: " + black);
        return black;
    }

    public double calibrateWhite() {
        double White;
        White = LightSensorCalibrator.getLightDetected();
        telemetry.log.add("White: " + White);
        return White;
    }


}

