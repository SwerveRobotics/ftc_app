package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Place robot near the mountain on
 * the blue side pointing at the beacon repair zone
 * and collects and spits out debris on the way
 */
@Autonomous(name="AutoBlueOutsideParkCollect")
public class AutoBlueOutsideParkCollect extends MasterAutonomous
{
    @Override public void main() throws InterruptedException
    {
        robotInit();
        configureTelemtry();

        waitForStart();
        initializeServoPositions();

        motorCollector.setPower(-POWER_FULL);
        driveForwardDistanceIMU(DRIVE_POWER, 4000);
        turnRightDegrees(DRIVE_POWER, 40);
        driveForwardDistanceIMU(DRIVE_POWER, 5000);
        stopDriving();
    }
}