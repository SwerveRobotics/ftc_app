package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.Gamepad;

/*
 * Robot attributes used in TeleOp programs
 */
public class MasterTeleOp extends Master
{
    double slowModeFactor = 1.0;
    boolean lightsAreOn = true;

    public void tankDrive(Gamepad gamepad)
    {
        // Tank drive based on joysticks of controller 1

        double leftRequestedPower = gamepad.left_stick_y;
        double rightRequestedPower = gamepad.right_stick_y;

        double acceleration = 0.1;

        // Slow mode
        if(gamepad.y)
        {
            slowModeFactor = 2.5;
        }
        else if(gamepad.a)
        {
            slowModeFactor = 1.0;
        }

        /*
        // Smooth acceleration
        if(leftRequestedPower > motorLeft.getPower())
        {
            leftRequestedPower = motorLeft.getPower() + acceleration;
        }
        else if(leftRequestedPower < motorLeft.getPower())
        {
            leftRequestedPower = motorLeft.getPower() - acceleration;
        }
        if(rightRequestedPower > motorRight.getPower())
        {
            rightRequestedPower = motorRight.getPower() + acceleration;
        }
        else if(rightRequestedPower < motorRight.getPower())
        {
            rightRequestedPower = motorRight.getPower() - acceleration;
        }*/

        motorLeft.setPower(leftRequestedPower);
        motorRight.setPower(rightRequestedPower);
    }

    public void controlTapeMeasureMotors(Gamepad gamepad)
    {
        // Tape Measure of Doom extension based on controller 1
        if(gamepad.right_trigger > 0)
        {
            motorTapeMeasure.setPower(-gamepad.right_trigger);
        }
        else if(gamepad.left_trigger > 0)
        {
            motorTapeMeasure.setPower(gamepad.left_trigger);
        }
        else
        {
            motorTapeMeasure.setPower(POWER_STOP);
        }
    }

    public void controlTapeMeasureServos(Gamepad gamepad)
    {
        // Tape Measure of Doom elevation based on controller 1
        if((gamepad1.dpad_up && servoTapeMeasureElevation.getPosition() <= 1 - TAPE_MEASURE_ELEVATION_RATE)
                || (gamepad2.dpad_up && servoTapeMeasureElevation.getPosition() <= 1 - TAPE_MEASURE_ELEVATION_RATE))
        {
            servoTapeMeasureElevation.setPosition(servoTapeMeasureElevation.getPosition() + TAPE_MEASURE_ELEVATION_RATE);
        }
        else if((gamepad1.dpad_down && servoTapeMeasureElevation.getPosition() >= 0 + TAPE_MEASURE_ELEVATION_RATE)
                || (gamepad2.dpad_down && servoTapeMeasureElevation.getPosition() >= 0 + TAPE_MEASURE_ELEVATION_RATE))
        {
            servoTapeMeasureElevation.setPosition(servoTapeMeasureElevation.getPosition() - TAPE_MEASURE_ELEVATION_RATE);
        }

        // Tape Measure of Doom locking
        if(gamepad.x && gamepad.b)
        {
            servoTapeMeasureLock.setPosition(TAPE_MEASURE_LOCK_POSITION);
        }
        else if(gamepad.start && gamepad.back)
        {
            servoTapeMeasureLock.setPosition(TAPE_MEASURE_UNLOCK_POSITION);
        }
    }

    public void controlCollector(Gamepad gamepad)
    {
        // Move collector based on triggers on controller 2 if the bottom isn't up
        if(gamepad.right_trigger > 0 && !collectorHingeIsUp)
            motorCollector.setPower(gamepad.right_trigger);
        else if(gamepad.left_trigger > 0 && !collectorHingeIsUp)
            motorCollector.setPower(-gamepad.left_trigger);
        else
            motorCollector.setPower(POWER_STOP);
    }

    public void controlScorer(Gamepad gamepad)
    {
        // Move scorer based on D-pad on controller 2
        if(gamepad.dpad_left)
            motorScorer.setPower(POWER_SCORER);
        else if(gamepad.dpad_right)
            motorScorer.setPower(-POWER_SCORER);
        else
            motorScorer.setPower(POWER_STOP);
    }

    public void controlZiplineServos(Gamepad gamepad)
    {
        // Toggle zipline servos based on left and right bumpers
        if (gamepad.left_bumper)
        {
            ziplineLeftIsOut = !ziplineLeftIsOut;
            if(ziplineLeftIsOut)
                servoLeftZipline.setPosition(ZIPLINE_LEFT_OUT);
            else
                servoLeftZipline.setPosition(ZIPLINE_LEFT_UP);
        }
        if(gamepad.right_bumper)
        {
            ziplineRightIsOut = !ziplineRightIsOut;
            if (ziplineRightIsOut)
                servoRightZipline.setPosition(ZIPLINE_RIGHT_OUT);
            else
                servoRightZipline.setPosition(ZIPLINE_RIGHT_UP);
        }
    }

    public void controlCollectorRamp(Gamepad gamepad)
    {
        // Move collector ramp up and down based on x and y
        if(gamepad.y)
        {
            collectorHingeIsUp = true;
            motorCollector.setPower(POWER_STOP);
            servoCollectorHinge.setPosition(COLLECTOR_HINGE_UP);
        }
        else if(gamepad.x)
        {
            collectorHingeIsUp = false;
            servoCollectorHinge.setPosition(COLLECTOR_HINGE_DOWN);
        }
    }

    public void controlClimberDumper(Gamepad gamepad)
    {
        // Toggle climber dumper servo position
        if (gamepad.a)
        {
            climberArmOut = !climberArmOut;
            if(climberArmOut)
                servoClimberDumper.setPosition(CLIMBER_ARM_OUT);
            else
                servoClimberDumper.setPosition(CLIMBER_ARM_IN);
        }
    }

    public void toggleLights(Gamepad gamepad)
    {
        if(gamepad.b)
        {
            lightsAreOn = !lightsAreOn;

            if(lightsAreOn)
                lights.setPower(POWER_FULL);
            else lights.setPower(POWER_STOP);
        }
    }
}
