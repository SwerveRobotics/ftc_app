package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Place robot near the mountain on
 * the red side pointing at the beacon repair zone
 * and collects and spits out debris on the way
 */
@Autonomous(name="AutoRedOutsideParkCollect")
public class AutoRedOutsideParkCollect extends MasterAutonomous
{
    @Override public void main() throws InterruptedException
    {
        robotInit();
        configureTelemtry();

        waitForStart();
        initializeServoPositions();

        motorCollector.setPower(-POWER_FULL);
        driveForwardDistanceIMU(DRIVE_POWER, 4000);
        turnLeftDegrees(DRIVE_POWER, 40);
        driveForwardDistanceIMU(DRIVE_POWER, 5000);
        stopDriving();
    }
}