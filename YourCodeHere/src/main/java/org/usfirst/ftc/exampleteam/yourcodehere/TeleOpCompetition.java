package org.usfirst.ftc.exampleteam.yourcodehere;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Main TeleOp file for 8923 bot
 */
@TeleOp(name="TeleOp Competition")
public class TeleOpCompetition extends MasterTeleOp
{
    @Override protected void main() throws InterruptedException
    {
        robotInit();
        configureTelemtry();

        // Wait for the game to begin
        this.waitForStart();
        initializeServoPositions();

        // Loop until the game is finished
        while(this.opModeIsActive())
        {
            if(this.updateGamepads())
            {
                driver1Controls();
                driver2Controls();
            }

            // Emit the latest telemetry and wait, letting other things run
            this.telemetry.update();
            this.idle();
        }
    }

    public void driver1Controls()
    {
        tankDrive(gamepad1);
        controlTapeMeasureMotors(gamepad1);
        controlTapeMeasureServos(gamepad1);
    }

    public void driver2Controls() throws InterruptedException
    {
        controlCollector(gamepad2);
        controlScorer(gamepad2);
        controlZiplineServos(gamepad2);
        controlCollectorRamp(gamepad2);
        controlClimberDumper(gamepad2);
        toggleLights(gamepad2);
    }
}
