package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Created by crowcasso on 10/14/15.
 */

@org.swerverobotics.library.interfaces.Autonomous
public class Autonomous extends SynchronousOpMode {

    private DcMotor motorR;
    private DcMotor motorL;
    final static int ENCODER_CPR = 1120;
    final static double GEAR_RATIO = 1;
    final static double WHEEL_DIAMETER = 4.9;
    final static double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    @Override
    protected void main() throws InterruptedException {   //Combined setup and run methods.
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        motorL.setDirection(DcMotor.Direction.REVERSE);
        motorL.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);
        motorR.setChannelMode(DcMotorController.RunMode.RESET_ENCODERS);

        motorL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        waitForStart();

        while (opModeIsActive()) {
            drive(0.5, 25);
            Thread.sleep(1000);
            turn(0.5, -90);
            Thread.sleep(5000);
        }

    }
    public int inchesToRotations(double distance)  //Same two methods.
    {
        double rotations = distance/CIRCUMFERENCE;
        return (int)(ENCODER_CPR * rotations * GEAR_RATIO);
    }

    public void drive(double power, double distance) throws InterruptedException {

        int n = inchesToRotations(distance);

        int start = motorR.getCurrentPosition();



        motorL.setPower(power);
        motorR.setPower(power);



        while (motorR.getCurrentPosition() < (start + n)) {}

        motorL.setPower(0.0);
        motorR.setPower(0.0);
    }
    public void turn(double power, double degree){

        int n = degreesToRotations(degree);
        int start = motorR.getCurrentPosition();

        double leftPower = power;
        double rightPower = power;

        if (degree > 0.0){
            leftPower *= -1;
        }
        else if (degree < 0.0){
            rightPower *= -1;
        }

        motorR.setPower(rightPower);
        motorL.setPower(leftPower);

        if(degree < 0) while (motorR.getCurrentPosition() > (start - n)) {}
        else while (motorR.getCurrentPosition() < (start + n)) {}

        motorR.setPower(0.0);
        motorL.setPower(0.0);

    }
    public int degreesToRotations(double dgr){

        return 800;
    }
}
