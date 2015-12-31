package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.Autonomous;
import org.swerverobotics.library.interfaces.IFunc;

/**
 * This simple OpMode illustrates how to drive autonomously a certain distance using encoders.
 *
 * The OpMode works with both legacy and modern motor controllers. It expects two motors,
 * named "motorLeft" and "motorRight".
 */

@Autonomous(name="DriveEncoders")
// @Disabled
        public class DriveEncoders extends SynchronousOpMode {
    //----------------------------------------------------------------------------------------------
    // Declare & Initialize Hardware
    //----------------------------------------------------------------------------------------------

    // The number of encoder ticks per motor shaft revolution.
    // 1440 HiTechnic motors.
    // 1120 Andy Mark Neverest motors
    // http://www.cougarrobot.com/index.php?option=com_content&view=article&id=331%3Aandymark-neverest-motor-notes&catid=92%3Aftc-hardware&Itemid=140

    DcMotor motorRight;
    DcMotor motorLeft;


    //----------------------------------------------------------------------------------------------
    // Main loop
    //----------------------------------------------------------------------------------------------

    @Override
    protected void main() throws InterruptedException {
        this.composeDashboard();

        this.motorLeft = this.hardwareMap.dcMotor.get("motorLeft");
        this.motorRight = this.hardwareMap.dcMotor.get("motorRight");

        this.motorLeft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

    //Autonomous Code below


    }//Main

    //----------------------------------------------------------------------------------------------
    // Utility - SubMethods
    //----------------------------------------------------------------------------------------------

    public void DriveEncoderDistance(double power, int distance){
        //reset encoders
        motorLeft.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        //Set Target
        motorLeft.setTargetPosition(distance);
        motorRight.setTargetPosition(distance);

        //Set Power
        DriveForward(power);

        while (motorLeft.isBusy() && motorRight.isBusy()){
            //wait until target is reached
        }

        //Stop and return mode to normal
        StopDriving();
        motorLeft.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);



    public void DriveForward(double power) {
        motorLeft.setPower(power);
        motorRight.setPower(power);
    }

    public void StopDriving() {
        DriveForward(0);
    }
}//DriveEncoder
