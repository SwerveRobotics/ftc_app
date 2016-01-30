package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.Disabled;

/**
 * Robot starts on red side, goes to beacon,
 * presses beacon button, and scores climber,
 * parks in floor goal
 */
@Autonomous(name="AutoRedInsideBeaconClimberZipliner")
@Disabled
public class AutoRedInsideBeaconClimberZipliner extends MasterAutonomous
{
    @Override public void main() throws InterruptedException
    {
        robotInit();
        configureTelemtry();

        waitForStart();
        initializeServoPositions();

        driveBackwardDistance(DRIVE_POWER, FOO);
        alignWithRedSideWhiteLine();
        correctDistanceToWall();

        readBeaconColors();
        // Decide which side is red, and set servo
        if(leftRed > rightRed + colorDifferenceThreshold && rightBlue > leftBlue + colorDifferenceThreshold)
            servoLeftZipline.setPosition(LEFT_BEACON_BUTTON_POSITION);
        else if(rightRed > leftRed + colorDifferenceThreshold && leftBlue > rightBlue + colorDifferenceThreshold)
            servoLeftZipline.setPosition(RIGHT_BEACON_BUTTON_POSITION);
        pressBeaconButton();

        dumpClimbers();
        driveForwardDistance(DRIVE_POWER, FOO);
        turnRightDistance(DRIVE_POWER, FOO);
        servoLeftZipline.setPosition(ZIPLINE_RIGHT_OUT);
        driveBackwardDistance(DRIVE_POWER, FOO);
        servoLeftZipline.setPosition(ZIPLINE_RIGHT_UP);
        stopDriving();
    }
}