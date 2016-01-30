package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotorController;

/*
 * Robot attributes used in autonomous programs
 */
public class MasterAutonomous extends Master
{
    // Variables for beacon colors
    int leftBlue = 0;
    int leftRed = 0;
    int rightBlue = 0;
    int rightRed = 0;
    int colorDifferenceThreshold = 300;

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

        while(Math.abs(motorLeft.getCurrentPosition()) < Math.abs(distance) || Math.abs(motorRight.getCurrentPosition()) < Math.abs(distance))
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

    public void driveForwardDistanceIMU(double power, int distance) throws InterruptedException
    {
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        double calibratedHeading = imu.getAngularOrientation().heading;
        double currentHeading = imu.getAngularOrientation().heading - calibratedHeading;
        double offsetMultiplier = 0.2;

        while(Math.abs(motorLeft.getCurrentPosition()) < Math.abs(distance) || Math.abs(motorRight.getCurrentPosition()) < Math.abs(distance))
        {
            // Use IMU to keep us driving straight
            currentHeading = imu.getAngularOrientation().heading - calibratedHeading;
            if(currentHeading > 180)
                currentHeading = currentHeading - 360;
            motorLeft.setPower(power + currentHeading * offsetMultiplier);
            motorRight.setPower(power - currentHeading * offsetMultiplier);

            // Wait until distance is reached
            telemetry.update();
            idle();
        }

        stopDriving();
    }

    public void driveBackwardDistanceIMU(double power, int distance) throws InterruptedException
    {
        driveForwardDistanceIMU(-power, -distance);
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

        while(Math.abs(motorLeft.getCurrentPosition()) < Math.abs(distance) || Math.abs(motorRight.getCurrentPosition()) < Math.abs(distance))
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

    public void turnRightDegrees(double power, int angle) throws InterruptedException
    {
        double calibratedHeading = imu.getAngularOrientation().heading;
        double currentHeading = imu.getAngularOrientation().heading - calibratedHeading;

        turnRight(power);

        while(Math.abs(currentHeading) < Math.abs(angle))
        {
            currentHeading = imu.getAngularOrientation().heading - calibratedHeading;
            telemetry.log.add("current" + currentHeading);
            if(currentHeading > 180)
                currentHeading = currentHeading - 360;
            // Wait until we've reached our target angle
            telemetry.log.add("current" + currentHeading + " actual" + formatNumber(imu.getAngularOrientation().heading));
            telemetry.update();
            idle();
        }

        stopDriving();
    }

    public void turnLeftDegrees(double power, int angle) throws InterruptedException
    {
        turnRightDegrees(-power, -angle);
    }

    public void dumpClimbers() throws InterruptedException
    {
        servoClimberDumper.setPosition(CLIMBER_DUMP_POSITION);
        delay(1500);
    }

    public void readBeaconColors() throws InterruptedException
    {
        // Check left side
        servoLeftZipline.setPosition(LEFT_BEACON_BUTTON_POSITION);
        delay(500);
        leftBlue = colorSensorBeacon.blue();
        leftRed = colorSensorBeacon.red();
        telemetry.log.add("Left blue:" + leftBlue + " red:" + leftRed);

        // Check right side
        servoLeftZipline.setPosition(RIGHT_BEACON_BUTTON_POSITION);
        delay(500);
        rightBlue = colorSensorBeacon.blue();
        rightRed = colorSensorBeacon.red();
        telemetry.log.add("Right blue:" + rightBlue + " red:" + rightRed);
    }

    public void pressBeaconButton() throws InterruptedException
    {
        driveBackwardDistance(DRIVE_POWER / 2, 200);
        driveForwardDistance(DRIVE_POWER / 2, 200);
    }

    public void alignWithBlueSideWhiteLine() throws InterruptedException
    {
        double whiteLineValue = 0.07;
        motorLeft.setPower(-DRIVE_POWER / 2);
        //motorRight.setPower(-DRIVE_POWER / 4.0);

        telemetry.log.add("Start:" + formatNumber(lightSensorBack.getLightDetected()));
        while(lightSensorBack.getLightDetected() < whiteLineValue)
        {

            telemetry.log.add("Checking back:" + formatNumber(lightSensorBack.getLightDetected()));
            // Wait until back light sensor detects line
            telemetry.update();
            idle();
        }
        whiteLineValue = 0.3;

        /*
        turnRight(DRIVE_POWER / 2);
        while(lightSensorFront.getLightDetected() > whiteLineValue)
        {
            telemetry.log.add("Checking front:" + formatNumber(lightSensorFront.getLightDetected()));
            // Wait until front light sensor detects line
            telemetry.update();
            idle();
        }*/
        stopDriving();
    }

    public void alignWithRedSideWhiteLine() throws InterruptedException {
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

    public void correctDistanceToWall() throws InterruptedException
    {
        driveForwardDistance(DRIVE_POWER / 2, (int) ((30.0 - getDistance()) * 100.0));
    }
}
