package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.swerverobotics.library.examples.SynchTelemetryOp;
import org.swerverobotics.library.interfaces.EulerAngles;
import org.swerverobotics.library.interfaces.IFunc;

/*
 *
 */
public class Master8923Autonomous extends Master8923
{
    public void servoStartingPositions()
    {
        servoClimberDumper.setPosition(CLIMBER_RETURN_POSITION);
        servoCollectorHinge.setPosition(COLLECTOR_HINGE_UP);
        servoLeftZipline.setPosition(ZIPLINE_LEFT_UP);
        servoRightZipline.setPosition(ZIPLINE_RIGHT_UP);
    }

    public void lightSensorLEDs (boolean state)
    {
        lightSensorBack.enableLed(state);
        lightSensorFront.enableLed(state);
    }

    public void driveForward(double power)
    {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void driveForwardDistance(double power, int distance) throws InterruptedException
    {
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);

        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        driveForward(power);

        while(Math.abs(motorLeft.getCurrentPosition()) < Math.abs(distance) && Math.abs(motorRight.getCurrentPosition()) < Math.abs(distance))
        {
            // Wait until distance is reached
            telemetry.update();
            idle();
        }

        stopDriving();

        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void driveBackwardDistance(double power, int distance) throws InterruptedException
    {
        driveForwardDistance(-power, -distance);
    }

    public void turnLeft(double power)
    {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
    }

    public void turnRight(double power)
    {
        turnLeft(-power);
    }

    public void stopDriving()
    {
        driveForward(0);
    }

    public void turnLeftDistance(double power, int distance) throws InterruptedException
    {
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorLeft.setTargetPosition(-distance);
        motorRight.setTargetPosition(distance);

        motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        turnLeft(power);

        while(Math.abs(motorLeft.getCurrentPosition()) < Math.abs(distance) && Math.abs(motorRight.getCurrentPosition()) < Math.abs(distance))
        {
            // Wait until distance is reached
            telemetry.update();
            idle();
        }

        stopDriving();

        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
    }

    public void turnRightDistance(double power, int distance) throws InterruptedException
    {
        turnLeftDistance(-power, distance);
    }

    public void TurnRightDegrees(double power, int angle) throws InterruptedException
    {
        double calibratedHeading = imu.getAngularOrientation().heading;
        double currentHeading = imu.getAngularOrientation().heading - calibratedHeading;

        turnRight(power);

        while(currentHeading > angle)
        {
            currentHeading = imu.getAngularOrientation().heading - calibratedHeading;
            // Wait until we've reached our target angle
            telemetry.update();
            idle();
        }

        stopDriving();
    }

    public void TurnLeftDegrees(double power, int angle) throws InterruptedException
    {
       TurnRightDegrees(-power, -angle);
    }

    public void dumpClimbers() throws InterruptedException
    {
        servoClimberDumper.setPosition(CLIMBER_DUMP_POSITION);
    }

    public void setRightZiplineOut() throws InterruptedException
    {
        servoRightZipline.setPosition(ZIPLINE_RIGHT_OUT);
    }

    public void setRightZiplineUp() throws InterruptedException
    {
        servoRightZipline.setPosition(ZIPLINE_RIGHT_UP);
    }

    public void setLeftZiplineOut() throws InterruptedException
    {
        servoLeftZipline.setPosition(ZIPLINE_LEFT_OUT);
    }

    public void setLeftZiplineUp() throws InterruptedException
    {
        servoLeftZipline.setPosition(ZIPLINE_LEFT_UP);
    }


    public void pressBeaconButton() throws InterruptedException
    {
        // Check for a range of blue
        if(colorSensorBeacon.blue() <= calibratedBlue + 50 && colorSensorBeacon.blue() < calibratedBlue - 50)
        {
            // Press Blue
            servoPressBeaconButton.setPosition(0.8);
            Thread.sleep(500);
            servoPressBeaconButton.setPosition(0.5);
        }
        else
        {
            // Otherwise press Red
            servoPressBeaconButton.setPosition(0.2);
            Thread.sleep(500);
            servoPressBeaconButton.setPosition(0.5);
        }

    }

    public void followLine() throws InterruptedException
    {
        while(lightSensorBack.getLightDetected() > 0.5)
        {
            driveForward(DRIVE_POWER);
        }
        stopDriving();
        while (lightSensorFront.getLightDetected() > 0.5)
        {
            motorRight.setPower(DRIVE_POWER);
        }
        stopDriving();
    }

    public void allignWithBlueSideWhiteLine() throws InterruptedException
    {
        driveForward(-DRIVE_POWER / 2);
        lightSensorLEDs(ON);
        while(lightSensorBack.getLightDetected() > 0.6)
        {
            // Wait until back light sensor detects line
            telemetry.update();
            idle();
        }
        turnRight(DRIVE_POWER / 2);
        while(lightSensorFront.getLightDetected() > 0.6)
        {
            // Wait until front light sensor detects line
            telemetry.update();
            idle();
        }
        /*
        while(lightSensorFront.getLightDetected() < 0.6)
        {
            // Wait until front light sensor detects line
            telemetry.update();
            idle();
        }
        */
    }

    public void alignWithRedSideWhiteLine() throws InterruptedException
    {
        driveForward(-DRIVE_POWER / 2);
        lightSensorLEDs(ON);
        while(lightSensorBack.getLightDetected() > 0.6)
        {
            // Wait until back light sensor detects line
            telemetry.update();
            idle();
        }
        turnLeft(DRIVE_POWER / 2);
        while(lightSensorFront.getLightDetected() > 0.6)
        {
            // Wait until front light sensor detects line
            telemetry.update();
            idle();
        }
        /*
        while(lightSensorFront.getLightDetected() < 0.6)
        {
            // Wait until front light sensor detects line
            telemetry.update();
            idle();
        }
        */
    }

    public double getDistance()
    {
        return ultrasonicSensor.getUltrasonicLevel();
    }

    public void configureTelemtry()
    {
        // Left drive motor info
        telemetry.addLine
                (
                        this.telemetry.item("Left Power:", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return motorLeft.getPower();
                            }
                        }),
                        this.telemetry.item("Left Position: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return motorLeft.getCurrentPosition();
                            }
                        })
                );

        // Right drive motor info
        telemetry.addLine
                (
                        this.telemetry.item("Right Power: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return motorRight.getPower();
                            }
                        }),
                        this.telemetry.item("Right Position: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return motorRight.getCurrentPosition();
                            }
                        })
                );

        // Light sensor info
        telemetry.addLine
                (
                        this.telemetry.item("Front light sensor: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return lightSensorFront.getLightDetected();
                            }
                        }),
                        this.telemetry.item("Back light sensor: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return lightSensorBack.getLightDetected();
                            }
                        })
                );

        telemetry.addLine
                (
                        this.telemetry.item("Ultrasonic: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return ultrasonicSensor.getUltrasonicLevel();
                            }
                        })
                );
    }
}