package org.usfirst.ftc.exampleteam.yourcodehere;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import org.swerverobotics.library.ClassFactory;
import org.swerverobotics.library.SynchronousOpMode;
import org.swerverobotics.library.interfaces.IBNO055IMU;
import org.swerverobotics.library.interfaces.IFunc;

/*
 * Global robot attributes used in all programs
 */
public class Master extends SynchronousOpMode
{
    DcMotor motorLeft = null;
    DcMotor motorRight = null;
    DcMotor motorCollector = null;
    DcMotor motorScorer = null;
    DcMotor motorTapeMeasure = null;
    DcMotor lights = null;

    Servo servoLeftZipline = null;
    Servo servoRightZipline = null;
    Servo servoTapeMeasureElevation = null;
    Servo servoTapeMeasureLock = null;
    Servo servoCollectorHinge = null;
    Servo servoClimberDumper = null;

    ColorSensor colorSensorBeacon;
    OpticalDistanceSensor lightSensorFront;
    OpticalDistanceSensor lightSensorBack;
    UltrasonicSensor ultrasonicSensor;
    IBNO055IMU imu;
    int calibratedBlue;

    //TODO change this
    int FOO = 1;

    // Declare constants
    double POWER_FULL = 1.0;
    double POWER_STOP = 0.0;
    double POWER_SCORER = 0.25 ;
    double ZIPLINE_LEFT_UP = 0.55;
    double ZIPLINE_LEFT_OUT = 0.2;
    double ZIPLINE_RIGHT_UP = 0.0;
    double ZIPLINE_RIGHT_OUT = 0.5;
    double LEFT_BEACON_BUTTON_POSITION = 0.6;
    double RIGHT_BEACON_BUTTON_POSITION = 0.5;
    double COLLECTOR_HINGE_DOWN = 0.63;
    double COLLECTOR_HINGE_UP = 0.8;
    double TAPE_MEASURE_START_POS = 0.8;
    double TAPE_MEASURE_ELEVATION_RATE = 0.05;
    double TAPE_MEASURE_UNLOCK_POSITION = 1.0;
    double TAPE_MEASURE_LOCK_POSITION = 0.0;
    double CLIMBER_ARM_OUT = 0.0;
    double CLIMBER_ARM_IN = 0.7;
    double DRIVE_POWER = -1.0;
    double CLIMBER_DUMP_POSITION = 1.0;
    double CLIMBER_RETURN_POSITION = 0.0;
    boolean ON = true;
    boolean OFF = false;

    // Declare variables
    boolean ziplineLeftIsOut = false;
    boolean ziplineRightIsOut = false;
    boolean collectorHingeIsUp = false;
    boolean climberArmOut = false;
    IBNO055IMU.Parameters   parameters = new IBNO055IMU.Parameters();

    @Override protected void main() throws InterruptedException {}

    public void robotInit()
    {
        // Initialize motors and servos
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorCollector = hardwareMap.dcMotor.get("motorCollector");
        motorScorer = hardwareMap.dcMotor.get("motorScorer");
        motorTapeMeasure = hardwareMap.dcMotor.get("motorTapeMeasure");
        lights = hardwareMap.dcMotor.get("lights");

        servoLeftZipline = hardwareMap.servo.get("servoLeftZipline");
        servoRightZipline = hardwareMap.servo.get("servoRightZipline");
        servoTapeMeasureElevation = hardwareMap.servo.get("servoTapeMeasureElevation");
        servoTapeMeasureLock = hardwareMap.servo.get("servoTapeMeasureLock");
        servoCollectorHinge = hardwareMap.servo.get("servoCollectorHinge");
        servoClimberDumper = hardwareMap.servo.get("servoClimberDumper");


        // Set motor channel modes
        motorLeft.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRight.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorCollector.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorScorer.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        // Reverse left motors so we don't spin in a circle
        motorRight.setDirection(DcMotor.Direction.REVERSE);

        /* Servos are not initialized to ensure we fit in the box
         * This is a hack to keep us inside the 18" limit
         * None of the servos move if none are set to a position
         * If any servo is set to a position, they all go
         * Initialize servo positions after the waitForStart()
         *
         * servoLeftZipline.setPosition(ZIPLINE_LEFT_UP);
         * servoRightZipline.setPosition(ZIPLINE_RIGHT_UP);
         * servoCollectorHinge.setPosition(COLLECTOR_HINGE_DOWN);
         * servoClimberDumper.setPosition(CLIMBER_ARM_IN);
         * servoTapeMeasureElevation.setPosition(TAPE_MEASURE_START_POS);
         * servoTapeMeasureLock.setPosition(TAPE_MEASURE_UNLOCK_POSITION);
         */

        // Initialize sensors
        colorSensorBeacon = hardwareMap.colorSensor.get("colorSensorBeacon");
        colorSensorBeacon.enableLed(false);
        lightSensorFront = hardwareMap.opticalDistanceSensor.get("lightSensorFront");
        lightSensorBack = hardwareMap.opticalDistanceSensor.get("lightSensorBack");
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("ultrasonicSensor");
        parameters.angleUnit = IBNO055IMU.ANGLEUNIT.DEGREES;
        imu = ClassFactory.createAdaFruitBNO055IMU(hardwareMap.i2cDevice.get("imu"), parameters);
    }

    public void initializeServoPositions()
    {
        // Initialize servos to starting positions
        servoLeftZipline.setPosition(ZIPLINE_LEFT_UP);
        servoRightZipline.setPosition(ZIPLINE_RIGHT_UP);
        servoCollectorHinge.setPosition(COLLECTOR_HINGE_DOWN);
        servoClimberDumper.setPosition(CLIMBER_ARM_IN);
        servoTapeMeasureElevation.setPosition(TAPE_MEASURE_START_POS);
        servoTapeMeasureLock.setPosition(TAPE_MEASURE_UNLOCK_POSITION);
    }

    public void configureTelemtry()
    {
        // Left drive motor info
        telemetry.addLine
                (
                        this.telemetry.item("Left Power: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(motorLeft.getPower());
                            }
                        }),
                        this.telemetry.item("Encoder: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(motorLeft.getCurrentPosition());
                            }
                        })
                );

        // Right drive motor info
        telemetry.addLine
                (
                        this.telemetry.item("Right Power: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(motorRight.getPower());
                            }
                        }),
                        this.telemetry.item("Encoder: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(motorRight.getCurrentPosition());
                            }
                        })
                );

        // Light sensor info
        telemetry.addLine
                (
                        this.telemetry.item("Front light sensor: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(lightSensorFront.getLightDetected());
                            }
                        }),
                        this.telemetry.item("Back light sensor: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return formatNumber(lightSensorBack.getLightDetected());
                            }
                        })
                );

        telemetry.addLine
                (
                        this.telemetry.item("Ultrasonic: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return ultrasonicSensor.getUltrasonicLevel();
                            }
                        })
                );

        telemetry.addLine
                (
                        this.telemetry.item("Heading: ", new IFunc<Object>() {
                            @Override
                            public Object value() {
                                return imu.getAngularOrientation().heading;
                            }
                        })
                );
    }

    public String formatNumber(double number)
    {
        return String.format("%.2f", number);
    }

    // Use this instead of Thread.sleep()
    public void delay(long millis) throws InterruptedException
    {
        long startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < millis)
        {
            telemetry.update();
            idle();
        }
    }
}