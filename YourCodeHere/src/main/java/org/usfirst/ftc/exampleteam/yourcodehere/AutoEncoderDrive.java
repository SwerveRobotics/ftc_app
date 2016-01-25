package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.IFunc;

/**
 * Example Autonomous OpMode illustrating how to DRIVE and TURN
 * using a certain distances using encoders.
 *
 * The OpMode works with both legacy and modern motor controllers. It expects two motors,
 * named "motorLeft" and "motorRight". And can be used with both HiTechnic and Neverest motors
 * Note: the encoder constant must be determined for your robot configuration (wheel size & gearing)
 *
 */

@Autonomous(name="AutoEncoderDrive")
// @Disabled
public class AutoEncoderDrive extends SynchronousOpMode {
    //----------------------------------------------------------------------------------------------
    // Declare Hardware
    //----------------------------------------------------------------------------------------------

    /** The number of encoder ticks per motor shaft revolution.
    // 1440 HiTechnic motors.
    // 1120 Andy Mark Neverest motors
    //http://www.cougarrobot.com/index.php?option=com_content&view=article&id=331%3Aandymark-neverest-motor-notes&catid=92%3Aftc-hardware&Itemid=140
    */

    DcMotor motorRight;
    DcMotor motorLeft;


    int constant = 1120;    /** This value is initially set to clicks for on rotation. and the desired distance of 1 inch
                            *   to determine encoder constant divide encRotations by the distance travelled in one full rotation
                            *   this will provide the encoder counts equal to 1 inch of travel.
                            */


    //----------------------------------------------------------------------------------------------
    // Main loop
    //----------------------------------------------------------------------------------------------

    @Override
    protected void main() throws InterruptedException {
        //-------------------------------------
        // Initialize Hardware
        //-------------------------------------

        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");

        this.motorRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //Autonomous Code goes here....
        //------------------------------------------------------------------------------------------

        //example driving code...
        DriveEncoderDistance(0.5, 1);   //(power, distance inches)
        LeftEncoderTurn(0.5, 2);
        DriveEncoderDistance(0.2, 1);
        RightEncoderTurn(0.5, 2);
        //add additional straight and turn methods to drive robot as desired

        StopDriving();
        //------------------------------------------------------------------------------------------


    }//Main

    //----------------------------------------------------------------------------------------------
    // Utility - SubMethods
    //----------------------------------------------------------------------------------------------

    public void DriveEncoderDistance(double power, int distance) throws InterruptedException{
        //reset encoders
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // waits for encoders to read zero
        while(motorLeft.getCurrentPosition() != 0 ||
                motorRight.getCurrentPosition() != 0) {
            //empty loop to wait until zero is reached
        }

        //calculate encoder clicks
        int COUNTS = distance * constant;

        //Set Target
        motorLeft.setTargetPosition(COUNTS);
        motorRight.setTargetPosition(COUNTS);

        //Set Power
        DriveForward(power);

        // Set them a-going
        this.motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeft.isBusy() && motorRight.isBusy()) {
            //Empty loop to wait until target is reached
        }

        //Stop and return mode to normal
        StopDriving();

        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }//DriveEncoderDistance

    // The basic Drive & Turn Methods
    public void DriveForward(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }
    public void DriveRight(double power) {
        motorLeft.setPower(-power);
        motorRight.setPower(power);
    }
    public void DriveLeft(double power) {
        DriveRight(-power);
    }
    public void StopDriving() {
        DriveForward(0);
    }
    //----------------------------------------------------------------------------------------------

    public void RightEncoderTurn(double power, int distance) throws InterruptedException{
        //reset encoders
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // waits for encoders to read zero
        while(motorLeft.getCurrentPosition() != 0 ||
                motorRight.getCurrentPosition() != 0) {
            //empty loop to wait until zero is reached
        }

        //calculate encoder clicks
        int COUNTS = distance * constant;

        //Set Target

        motorRight.setTargetPosition(COUNTS);
        motorLeft.setTargetPosition(-COUNTS);

        //Set Power
        DriveRight(power);

        // Set them a-going
        this.motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeft.isBusy() && motorRight.isBusy()) {
            //Empty loop to wait until target is reached
        }
        //Stop and return mode to normal
        StopDriving();
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }//RightEncoderTurn


    //----------------------------------------------------------------------------------------------

    public void LeftEncoderTurn(double power, int distance) throws InterruptedException{
        //reset encoders
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        // waits for encoders to read zero
        while(motorLeft.getCurrentPosition() != 0 ||
                motorRight.getCurrentPosition() != 0) {
            //empty loop to wait until zero is reached
        }

        //calculate encoder clicks
        int COUNTS = distance * constant;

        //Set Target
        motorRight.setTargetPosition(-COUNTS);
        motorLeft.setTargetPosition(COUNTS);

        //Set Power
        DriveLeft(power);

        // Set them a-going
        this.motorLeft.setMode(DcMotorController.RunMode.RUN_TO_POSITION);
        this.motorRight.setMode(DcMotorController.RunMode.RUN_TO_POSITION);

        while (motorLeft.isBusy() && motorRight.isBusy()) {
            //Empty loop to wait until target is reached
        }

        //Stop and return mode to normal
        StopDriving();
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

    }//LeftEncoderTurn

}//AutoEncoderDrive


