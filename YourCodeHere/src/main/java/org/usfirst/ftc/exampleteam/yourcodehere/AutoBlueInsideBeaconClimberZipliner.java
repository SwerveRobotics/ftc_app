package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Robot starts on blue side, goes to beacon,
 * presses beacon button, and scores climber,
 * parks in floor goal
 */
@Autonomous(name="AutoBlueInsideBeaconClimberZipliner")
@Disabled
public class AutoBlueInsideBeaconClimberZipliner extends MasterAutonomous
{
    @Override public void main() throws InterruptedException
    {
        robotInit();
        configureTelemtry();
        telemetry.setUpdateIntervalMs(50);
        lightSensorLEDs(ON);

        waitForStart();
        initializeServoPositions();

        // This is a hack to keep us inside the 18" limit
        // None of the servos move if none are set to a position
        // If any servo is set to a position, they all go
        //servoStartingPositions();

        //driveBackwardDistanceIMU(DRIVE_POWER, 11500);
        alignWithBlueSideWhiteLine();
        /*
        correctDistanceToWall();

        readBeaconColors();
        // Decide which side is blue, and set servo to correct side
        if(leftBlue > rightBlue + colorDifferenceThreshold && rightRed > leftRed + colorDifferenceThreshold)
            servoLeftZipline.setPosition(LEFT_BEACON_BUTTON_POSITION);
        else if(rightBlue > leftBlue + colorDifferenceThreshold && leftRed > rightRed + colorDifferenceThreshold)
            servoLeftZipline.setPosition(RIGHT_BEACON_BUTTON_POSITION);
        pressBeaconButton();

        dumpClimbers();
        driveForwardDistance(DRIVE_POWER, FOO);
        turnLeftDistance(DRIVE_POWER, FOO);
        servoRightZipline.setPosition(ZIPLINE_RIGHT_OUT);
        driveBackwardDistance(DRIVE_POWER, FOO);
        servoRightZipline.setPosition(ZIPLINE_RIGHT_UP);
        stopDriving();*/
    }
}