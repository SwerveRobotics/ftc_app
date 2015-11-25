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

    // Hardware
    private DcMotor motorR;
    private DcMotor motorL;
    private DcMotor sweeper;
    private DcMotor arm;
    private Servo wrist;
    private TouchSensor limit;

    @Override
    public void init() {

        /**
         **************************************************
         *     Robot Setup                                *
         **************************************************
         * VG0E Motor Controller - 1) MotorR   2) MotorL  *
         * YCGG Motor Controller - 1) arm      2) sweeper *
         * VD4U Device Interface Module - 0) limit        *
         * VDLE Servo Controller - 1) wrist               *
         **************************************************
         **/


        // Hardware Map
        motorR = hardwareMap.dcMotor.get("motorR");
        motorL = hardwareMap.dcMotor.get("motorL");
        arm = hardwareMap.dcMotor.get("arm");
        wrist = hardwareMap.servo.get("wrist");
        limit = hardwareMap.touchSensor.get("limit");
        sweeper = hardwareMap.dcMotor.get("sweeper");

        // Drive Base Setup
        motorL.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorR.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        motorL.setDirection(DcMotor.Direction.REVERSE);

        // Arm Setup
        arm.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        arm.setDirection(DcMotor.Direction.REVERSE);

        // Starting Values
        wristPosition = STARTING_WRIST_POSITION;

        armPosition = 0.0;
        armState = ArmState.RESTING_DOWN;   // start robot with arm down

        stopTime = 0;
    }

    @Override
    public void loop() {
        stateOfTheArm();        // check the state of the arm
        manualArmControl();     // manually move the arm (or not)
        //autoArmControl();     // automatically move the arm (or not)
        driveControl();         // drive the robot (or not)
        wristControl();         // move the bucket (or not)
        sweeperControl();       // run the sweeper (or not)
    }


    /**
     ******************************
     *            ARM             *
     ******************************
     **/

    private final double ARM_DOWN_SPEED = -0.60;
    private final double ARM_UP_SPEED = 0.60;
    private final double ARM_SLOW_SPEED = 0.30;
    private final double TRIGGER_THRESHOLD = 0.3;
    private final long BUCKET_EXTRACT_TIME = 1000;
    private final int ARM_MAX_POSITION = 6500;
    private final int ARM_MIN_POSITION = -200;
    private final int ARM_SLOW_POSITION = ARM_MAX_POSITION - 500;

    private double armPosition;
    private boolean autoArmUp = false;
    private boolean autoArmDown = false;
    private enum ArmState {RESTING_DOWN, MOVING_UP, MOVING_DOWN, RESTING_UP};
    private ArmState armState;
    private long stopTime;

    private boolean prevButPress = false;
    private boolean prevG1Bump = false;
    private boolean prevG1Trig = false;
    private boolean prevG2A = false;
    private boolean prevG2B;


    /**
     * Maintain the current start of the arm.
     *
     * ArmState.RESTING_DOWN  ==> arm is all the way down
     * ArmState.MOVING_DOWN   ==> arm last seen moving up
     * ArmState.MOVING_UP     ==> arm last seen moving down
     * ArmState.RESTING_UP    ==> arm is all the way up
     */
    private void stateOfTheArm() {
        if( (gamepad1.left_bumper && !prevG1Bump) || (gamepad2.a && !prevG2A) ) {
            if(armState == ArmState.RESTING_DOWN || armState == ArmState.MOVING_DOWN) {
                armState = ArmState.MOVING_UP;
            }
            else {
                armState = ArmState.MOVING_DOWN;
            }
        }
        if( (gamepad1.left_trigger > STICK_THRESHOLD && !prevG1Trig) || (gamepad2.b && !prevG2B) ) {
            if(armState == ArmState.RESTING_DOWN) {

            }
            if(armState == ArmState.RESTING_UP) {
                if(isWristUp) {
                    wristPosition = STARTING_WRIST_POSITION;
                    isWristUp = false;
                } else {
                    wristPosition = ENDING_WRIST_POSITION;
                    isWristUp = true;
                }
            }
        }

        prevG1Bump = gamepad1.left_bumper;
        prevG1Bump = (gamepad1.left_trigger > STICK_THRESHOLD);
        prevG2A = gamepad2.a;
        prevG2B = gamepad2.b;

        // Is the arm all the way down?
        if (armState == ArmState.MOVING_DOWN && limit.isPressed()) {
            armState = ArmState.RESTING_DOWN;
        }
    }

    /**
     * Controlling angle of the arm
     *
     * ONLY WORKS FOR CONTROLLER 1
     *
     * Left Bumper  ==>  bring arm up
     * Left Trigger ==>  bring arm down
     */
    private void manualArmControl() {

        if (gamepad1.left_bumper){
            // move the arm up
            arm.setPower(ARM_UP_SPEED);
            armState = ArmState.MOVING_UP;
            autoArmUp = false;
            autoArmDown = false;
            telemetry.addData("Left Bumper: ","Yes");
        }
        else if (gamepad1.left_trigger > TRIGGER_THRESHOLD && limit.isPressed() == false){
            // move the arm down
            arm.setPower(ARM_DOWN_SPEED);
            armState = ArmState.MOVING_DOWN;
            autoArmUp = false;
            autoArmDown = false;
        }
        else if (limit.isPressed() && armState != ArmState.MOVING_UP) {
            // arm is all the way down
            armState = ArmState.RESTING_DOWN;
            arm.setPower(0.0);
            autoArmUp = false;
            autoArmDown = false;
        }
        else {
            // arm does nothing for now
            arm.setPower(0);
        }

        telemetry.addData("Arm Position: ", arm.getCurrentPosition());
    }

    /**
     * Automatically move arm and bucket for scoring
     *
     * ONLY WORKS FOR CONTROLLER 2
     * (arm must be down in resting position)
     *
     * A button  ==> move arm and bucket up to scoring position
     */
    private void autoArmControl() {

        if (armState == ArmState.RESTING_DOWN && gamepad2.b){
            // flip the buckets up
            wristPosition = 0.0;
        }

        if (armState == ArmState.RESTING_DOWN && gamepad2.a){
            // start the automated arm/bucket movement
            autoArmUp = true;
            armState = ArmState.MOVING_UP;
            stopTime = System.currentTimeMillis() + 240000;     // 4 minutes in the future
            if (wristPosition != 0.0) {
                wristPosition = 0.0;
                stopTime = System.currentTimeMillis() + BUCKET_EXTRACT_TIME;
            }
            else{
                arm.setPower(ARM_UP_SPEED);
            }
        }

        if (armState == ArmState.RESTING_UP && gamepad2.a){
            // start the automatic downward movement
            autoArmDown = true;
        }

        if (autoArmUp) {
            // currently performing automated arm/bucket movement

            // wait some time for the bucket to extract
            long now = System.currentTimeMillis();
            if (now >= stopTime) {
                // bucket is hopefully out, start moving the arm
                arm.setPower(ARM_UP_SPEED);
            }

            armPosition = arm.getCurrentPosition();

            // angle the bucket so nothing falls out
            wristPosition = (armPosition - 2000) / 3000;
            wristPosition = Range.clip(wristPosition, 0, 1);

            if (armPosition >= ARM_SLOW_POSITION) {
                arm.setPower(ARM_SLOW_SPEED);
            }

            // stop arm at the upper limit
            if (armPosition >= ARM_MAX_POSITION) {
                arm.setPower(0);
                armState = ArmState.RESTING_UP;
                autoArmUp = false;
                isWristUp = true;
            }
        }

        if (autoArmDown) {
            // currently performing automated arm/bucket movement
            armPosition = arm.getCurrentPosition();

            // angle the bucket so nothing falls out
            wristPosition = (armPosition - 2000) / 3000;
            wristPosition = Range.clip(wristPosition, 0, 1);

            // stop arm at the upper limit
            if (limit.isPressed() || armPosition <= ARM_MIN_POSITION) {
                arm.setPower(0);
                armState = ArmState.RESTING_DOWN;
                arm.setMode(DcMotorController.RunMode.RESET_ENCODERS);
                autoArmDown = false;
                isWristUp = false;
                wristPosition = STARTING_WRIST_POSITION;
            }
        }

        telemetry.addData("autoArm: ", autoArmUp);
    }


    /**
     ******************************
     *          DRIVING           *
     ******************************
     **/


    private final double NORMAL_DRIVE_SPEED = 0.5;
    private final double LOW_DRIVE_SPEED = 0.3;
    private final double HIGH_DRIVE_SPEED = 1.0;
    private final double STICK_THRESHOLD = 0.05;

    /**
     * Drive the robot!
     */
    private void driveControl() {
        // assume controller 1 is driving
        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.right_stick_x;

        double speedMult = NORMAL_DRIVE_SPEED;
        int watcher = 0;

        if (gamepad1.right_bumper) {
            speedMult = LOW_DRIVE_SPEED;
            watcher = 1;
        }
        else if (gamepad1.right_trigger > TRIGGER_THRESHOLD) {
            speedMult = HIGH_DRIVE_SPEED;
            watcher = 2;
        }

        if (throttle < STICK_THRESHOLD && throttle > -STICK_THRESHOLD
                && direction < STICK_THRESHOLD && direction > -STICK_THRESHOLD) {
            throttle = gamepad2.left_stick_y;
            direction = gamepad2.right_stick_x;
            if (gamepad2.right_bumper) {
                speedMult = LOW_DRIVE_SPEED;
                watcher = 3;
            }
            else if (gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                speedMult = HIGH_DRIVE_SPEED;
                watcher = 4;
            }
        }

        // computer motor power
        float right = throttle - direction;
        float left = throttle + direction;

        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // set the motors
        motorR.setPower(right * speedMult);
        motorL.setPower(left * speedMult);

        // motor telemetry
        telemetry.addData("speedMult: ", speedMult);
        telemetry.addData("WATCHER", watcher);
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }


    /**
     ******************************
     *          SWEEPER           *
     ******************************
     **/


    private final double NORMAL_SWEEP_SPEED = 0.5;

    /**
     * Control the sweeper
     *
     * ONLY WORKS FOR CONTROLLER 1
     *
     * DPad down  ==> reverse sweeper (half-power)
     * DPad up    ==> forward sweeper (half-power)
     * Dpad left  ==> stop sweeper
     * DPad right ==> stop sweeper
     */
    private void sweeperControl() {
        double speed = NORMAL_SWEEP_SPEED;

        //Forward
        if (gamepad1.dpad_down) {
            sweeper.setPower(-speed);
        }

        //Stopped
        if (gamepad1.dpad_left || gamepad1.dpad_right){
            sweeper.setPower(0);
        }

        //Backward
        if (gamepad1.dpad_up) {
            sweeper.setPower(speed);
        }
    }


    /**
     ******************************
     *           WRIST            *
     ******************************
     **/


    private final double REG_WRIST_SPEED = 0.1;
    private final double FINE_WRIST_SPEED = 0.01;

    private double wristPosition;
    private boolean isWristUp = false;
    private final double STARTING_WRIST_POSITION = 0.3;
    private final double ENDING_WRIST_POSITION = 1.0;


    /**
     * Controlling angle of the bucket servo
     *
     * ONLY WORKS FOR CONTROLLER 1
     *
     * A button  ==>  bring bucket down
     * Y button  ==>  bring bucket up
     * B button  ==>  begin fine control
     * X button  ==>  end fine control
     */
    private void wristControl() {

        double amount = REG_WRIST_SPEED;

        // fine or normal control?
        if (gamepad1.x) {
            amount = REG_WRIST_SPEED;
        } else if (gamepad1.b) {
            amount = FINE_WRIST_SPEED;
        }


        if (gamepad1.a) {
            if (!prevButPress) {    // first time pressing A
                // bucket will move up
                if(wristPosition - amount >= 0) {
                    wristPosition -= amount;
                }
            }
            prevButPress = true;    // force release of A

        } else if (gamepad1.y) {
            if (!prevButPress) {    // first time pressing Y
                // move bucket down
                if(wristPosition + amount <= 1) {
                    wristPosition += amount;
                }
            }
            prevButPress = true;    // force release of Y

        } else {
            // we let up on either A or Y
            prevButPress = false;
        }

        // now ... actually move the bucket
        wrist.setPosition(wristPosition);

        telemetry.addData("Wrist Position: ", wristPosition);
    }


    /**
     ******************************
     *         UTILITY            *
     ******************************
     **/


    /**
     * Scale the speed of the drive motors to allow for
     * fine control and speed.
     *
     * @param dVal value to sacale
     * @return scaled value
     */
    private double scaleInput(double dVal)  {
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