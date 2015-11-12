package org.usfirst.ftc.aperturescience;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.swerverobotics.library.interfaces.TeleOp;

/**
 * Nasic 6.0 - TeleOp
 *
 * @author FTC 5064 Aperture Science
 */
@TeleOp
public class Nasic6_2 extends OpMode {

    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    private DcMotor arm;
    private Servo wrist;
    private TouchSensor limit;
    private boolean prevButPress = false;
    private double amount = .1;
    double wristPosition = .5;
    double armPosition = 0.0;
    boolean autoArm = false;
    boolean prevG1Bump = false, prevG1Trig = false, prevG2A = false, prevG2B, isWristUp = false;
    long stopTime;


    private enum ArmState {RESTING_DOWN, MOVING_UP, MOVING_DOWN, RESTING_UP};
    private ArmState armState;

    @Override
    public void init() {
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        arm = hardwareMap.dcMotor.get("arm");
        wrist = hardwareMap.servo.get("wrist");
        motorL.setDirection(DcMotor.Direction.REVERSE);
        limit = hardwareMap.touchSensor.get("limit");
        motorL.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setChannelMode(DcMotorController.RunMode.RUN_USING_ENCODERS);

        sweeper = hardwareMap.dcMotor.get("sweeper");
        arm.setDirection(DcMotor.Direction.REVERSE);

        wristPosition = 0.3;
        armState = ArmState.RESTING_DOWN;
        stopTime = 0;
    }

    @Override
    public void loop() {


        //////////////////
        //   Updators   //
        //////////////////

        if( (gamepad1.left_bumper && !prevG1Bump) || (gamepad2.a && !prevG2A) ) {
            if(armState == ArmState.RESTING_DOWN || armState == ArmState.MOVING_DOWN) armState = ArmState.MOVING_UP;
            else armState = ArmState.MOVING_DOWN;
        }
        if( (gamepad1.left_trigger > .3 && !prevG1Trig) || (gamepad2.b && !prevG2B) ) {
            if(armState == ArmState.RESTING_DOWN) {

            }
            if(armState == ArmState.RESTING_UP) {
                if(isWristUp) {
                    wristPosition = .35;
                    isWristUp = false;
                } else {
                    wristPosition = 1;
                    isWristUp = true;
                }
            }
        }

        prevG1Bump = gamepad1.left_bumper;
        prevG1Bump = (gamepad1.left_trigger > .3);
        prevG2A = gamepad2.a;
        prevG2B = gamepad2.b;

        ////////////////////
        //   Validators   //
        ////////////////////

        if (armState == ArmState.MOVING_DOWN && limit.isPressed()) {
            armState = ArmState.RESTING_DOWN;
        }


        /////////////////////
        //   Declarators   //
        /////////////////////

        if (gamepad1.left_bumper){
            arm.setPower(0.25);
            armState = ArmState.MOVING_UP;
            autoArm = false;
        }
        else if (gamepad1.left_trigger > 0.3f && limit.isPressed() == false){ //------------------------------------------------------------------
            arm.setPower(-0.25);
            armState = ArmState.MOVING_DOWN;
            autoArm = false;
        }
        else if (limit.isPressed() && armState != ArmState.MOVING_UP) {
            armState = ArmState.RESTING_DOWN;
            arm.setPower(0.0);
            autoArm = false;
        }
        else{
            arm.setPower(0.0);
        }

        if (armState == ArmState.RESTING_DOWN && gamepad2.a){
            autoArm = true;
            armState = ArmState.MOVING_UP;
            wristPosition = 0.0;
            stopTime = System.currentTimeMillis() + 1000;
        }

        if (autoArm) {
            long now = System.currentTimeMillis();
            if (now >= stopTime) {
                arm.setPower(0.25);
            }
            if (armState == ArmState.MOVING_UP) {
                armPosition = arm.getCurrentPosition();
                /*
                if (armPosition >= 2000) {
                    wristPosition = 1.0;
                }
                if (armPosition >= 4500) {
                    wristPosition = 0.0;
                }
                if (armPosition >= 6000) {
                    arm.setPower(0.0);
                    autoArm = false;
                }
                */
                wristPosition = (armPosition - 2000) / 3000;
                if(wristPosition < 0) wristPosition = 0;
                else if(wristPosition > 1) wristPosition = 1;
                if (armPosition >= 6500) {
                    arm.setPower(0.0);
                    autoArm = false;
                    armState = ArmState.RESTING_UP;
                    isWristUp = true;
                }
            }
        }



        /////////////////
        //   Driving   //
        /////////////////

        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.right_stick_x;
        double speedMult = .5;
        if (gamepad1.right_bumper) speedMult = .3;
        else if (gamepad1.right_trigger > .3) speedMult = 1;
        else speedMult = .5;
        if (throttle < .05 && throttle > -.05 && direction < .05 && direction > -.05) {
            throttle = gamepad2.left_stick_y;
            direction = gamepad2.right_stick_x;
            if (gamepad2.right_bumper) speedMult = .3;
            else if (gamepad2.right_trigger > .3) speedMult = 1;
            else speedMult = .5;
        }

        float right = throttle - direction;
        float left = throttle + direction;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        motorR.setPower(right * speedMult);
        motorL.setPower(left * speedMult);


        //wrist
        if (gamepad1.x) {
            amount = .1;
        } else if (gamepad1.b) {
            amount = .01;
        }

        if (gamepad1.a) {
            if (!prevButPress) {
                if(wristPosition - amount> 0) {
                    wristPosition -= amount;
                }
            }
            prevButPress = true;

        } else if (gamepad1.y) {
            if (!prevButPress) {
                if(wristPosition + amount < 1) {
                    wristPosition += amount;
                }
            }
            prevButPress = true;

        } else {
            prevButPress = false;
        }

        wrist.setPosition(wristPosition);

        // sweeper
        if (gamepad1.dpad_down) {
            sweeper.setPower(-0.5);
        }
        if (gamepad1.dpad_left || gamepad1.dpad_right){
            sweeper.setPower(0);
        }
        if (gamepad1.dpad_up) {
            sweeper.setPower(0.5);
        }

        //telemetry
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "left  pwr: "
                + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: "
                + String.format("%.2f", right));
        telemetry.addData("Wrist Position: ",wristPosition);
        telemetry.addData("Arm Position: ",arm.getCurrentPosition());
        telemetry.addData("autoArm: ", autoArm);
    }

    double scaleInput(double dVal)  {
        double[] scaleArray = {
                0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00
        };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * (scaleArray.length - 1));
        if (index < 0) {
            index = -index;
        } else if (index > (scaleArray.length - 1)) {
            index = scaleArray.length - 1;
        }

        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }
}
