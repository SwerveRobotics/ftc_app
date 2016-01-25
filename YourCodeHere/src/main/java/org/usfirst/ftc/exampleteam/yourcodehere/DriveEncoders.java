package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.IFunc;

/**
 * This is an example OpMode illustrating how to drive autonomously a certain distance using encoders.
 *
 * The OpMode works with both legacy and modern motor controllers. It expects two motors,
 * named "motorLeft" and "motorRight". And can be used with both HiTechnic and Neverest motors
 * Note: the encoder constant must be determined for your robot configuration (wheel size & gearing)
 *
 */

@Autonomous(name="DriveEncoders") // adds to registry file for name to appear on driver station
// @Disabled
        public class DriveEncoders extends SynchronousOpMode {
    //----------------------------------------------------------------------------------------------
    // Declare Hardware
    //----------------------------------------------------------------------------------------------

    /** The number of encoder ticks per motor shaft revolution.
    // 1440 HiTechnic motors.
    // 1120 Andy Mark Neverest motors
    // documentation found at here http://www.cougarrobot.com/index.php?option=com_content&view=article&id=331%3Aandymark-neverest-motor-notes&catid=92%3Aftc-hardware&Itemid=140
    */

    DcMotor motorRight;
    DcMotor motorLeft;

    int encRotations = 1120;
    int constant = 1234;    /** to determine encoder constant divide encRotations by the distance travelled in one full rotation
                            *   this will provide the encoder counts equal to 1 inch of travel.
 *                          */

    //----------------------------------------------------------------------------------------------
    // Main loop
    //----------------------------------------------------------------------------------------------

    @Override
    protected void main() throws InterruptedException {
        //-------------------------------------
        // Initialize Hardware.  Must match names given in configuration file on RC app
        //-------------------------------------

        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");

        this.motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //Autonomous Code goes here....
        DriveEncoderDistance(0.5,10);

    }//Main

    //----------------------------------------------------------------------------------------------
    // Utility - SubMethods
    //----------------------------------------------------------------------------------------------
    

    public void DriveEncoderDistance(double power, int distance) throws InterruptedException{    //method to drive set power and distance using encoders

        //reset encoders
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // waits for encoders to read zero
        while(motorLeft.getCurrentPosition() != 0 ||
                motorRight.getCurrentPosition() != 0) {
            //empty loop waits to allow encoders to reach zero before moving on
        }

        //calculate encoder clicks. Takes desired distance of inches and multiplies by constant
        int COUNTS = distance * constant;

        //Set Target
        motorLeft.setTargetPosition(COUNTS);
        motorRight.setTargetPosition(COUNTS);

        //Set Power
        DriveForward(power);

        // turns motors on
        this.motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeft.isBusy() && motorRight.isBusy()) {
            //wait until target is reached
        }

        //Stop and return mode to normal
        StopDriving();

        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }//DriveEncoderDistance
    public void DriveForward(double power) {    //sets both motors to desired power
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    public void StopDriving() {     //sets both motors to zero power
        DriveForward(0);
    }

}//DriveEncoder
