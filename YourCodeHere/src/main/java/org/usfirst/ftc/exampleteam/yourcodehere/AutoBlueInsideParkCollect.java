package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.interfaces.Autonomous;

/**
 * Place robot near the middle of the field on
 * the blue side pointing at the beacon repair zone
 * and collects and spits out debris on the way
 */
@Autonomous(name="AutoBlueInsideParkCollect")
public class AutoBlueInsideParkCollect extends MasterAutonomous
{
    @Override public void main() throws InterruptedException
    {
        robotInit();
        configureTelemtry();

        waitForStart();
        initializeServoPositions();

        motorCollector.setPower(-POWER_FULL);
        driveForwardDistanceIMU(DRIVE_POWER, 13000);
        stopDriving();
    }
}