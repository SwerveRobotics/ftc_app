package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Fusion on 11/8/2015.
 */
public class NeutrinoAuto extends LinearOpMode {
    DcMotor leftUpDrive;
    DcMotor leftDownDrive;
    DcMotor rightUpDrive;
    DcMotor rightDowndrive;
    GyroSensor sensorGyro;
    Servo climber;
    Servo buttonpusher;
    double gyroCalibrate;
    ColorSensor colorSensor;
    String teamColor = "red";
    String[] startPositions = {"corner", "mid", "mountain"};
    int startPosition = 1;
    int waitAtStart = 0;
    boolean pressButton = true;
    int waitAtButton = 0;
    boolean blockOtherButton = false;
    String[] upMountains = {"red", "blue", "farRed", "farBlue"};
    int upMountain = 0;
    boolean mountainClimbers = true;
    int menuChoice = 1;
    boolean startPress = false;
    boolean backPress = false;
    boolean yPress = false;
    boolean aPress = false;

    public double calcAverage(double[] vals) {
        double val = 0.0;
        int index = 0;
        while (index < vals.length) {
            index = index + 1;
        }

        return val/vals.length;
    }

    public void spinTurn (double ispeed, int idist) throws InterruptedException {
        telemetry.addData("spinturn started",idist);
        sensorGyro.resetZAxisIntegrator();
        while(sensorGyro.getHeading() > 0) {
            waitOneFullHardwareCycle();
        }

        int offset = 0;
        if (ispeed > 0.0) {
            offset = 360;
        }

        leftDownDrive.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        leftUpDrive.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightDowndrive.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightUpDrive.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        waitOneFullHardwareCycle();

        leftDownDrive.setPower(ispeed);
        rightDowndrive.setPower(-1.0 * ispeed);
        leftUpDrive.setPower(ispeed);
        rightUpDrive.setPower(-1.0 * ispeed);

        while ((Math.abs(offset - sensorGyro.getHeading()) < idist)||sensorGyro.getHeading()==0) {

            waitForNextHardwareCycle();

        }
        leftDownDrive.setPower(0.0);
        rightDowndrive.setPower(0.0);
        rightUpDrive.setPower(0.0);
        leftUpDrive.setPower(0.0);
        telemetry.addData("spinturn end",idist);
    }

    public void drive (double ispeed, int idist) throws InterruptedException {
        telemetry.addData("drive started",idist);
        telemetry.addData("color blue", colorSensor.blue());
        telemetry.addData("color red" , colorSensor.red());
        telemetry.addData("color green", colorSensor.green());

        double leftdownpos;
        double leftuppos;
        double rightuppos;
        double rightdownpos;
        double averagepos;
        double vSpeed = ispeed;
        //double[] motorvals;

        leftDownDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        leftUpDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightDowndrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightUpDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        while(leftDownDrive.getCurrentPosition() != 0 ||
                leftUpDrive.getCurrentPosition() != 0 ||  // waits for enc to be zero
                rightDowndrive.getCurrentPosition() != 0 ||
                rightUpDrive.getCurrentPosition() != 0){
            waitOneFullHardwareCycle();
        }
        //might need to be set target position
        telemetry.addData("after reset", vSpeed);
        leftDownDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        leftUpDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightDowndrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        rightUpDrive.setMode(DcMotorController.RunMode.RUN_USING_ENCODERS);
        int numDegrees = (int)(idist * 82.0);  // numDegrees is the encoder ticks

        // this part adjust the power to make it drive straight
        rightUpDrive.setPower(vSpeed);
        rightDowndrive.setPower(vSpeed);
        if (vSpeed < 0.0){
            vSpeed = vSpeed +0.5;
        } else {
            vSpeed = vSpeed - 0.5;
        }
        leftUpDrive.setPower(vSpeed);
        leftDownDrive.setPower(vSpeed);
        telemetry.addData("before while", vSpeed);

        // use while instead of setTarget
        while (Math.abs(leftDownDrive.getCurrentPosition()) < numDegrees) {
            /*rightdownpos=rightDowndrive.getCurrentPosition();
            rightuppos=rightUpDrive.getCurrentPosition();
            leftdownpos=leftDownDrive.getCurrentPosition();
            leftuppos=leftUpDrive.getCurrentPosition();
            double[] motorvals={rightdownpos,rightuppos,leftdownpos,leftuppos};
            averagepos=calcAverage(motorvals);
            if (Math.abs(averagepos)>0.0) {
                leftDownDrive.setPower(leftDownDrive.getPower() * (averagepos / leftdownpos));
                leftUpDrive.setPower(leftUpDrive.getPower() * (averagepos / leftuppos));
                rightDowndrive.setPower(rightDowndrive.getPower() * (averagepos / rightdownpos));
                rightUpDrive.setPower(rightUpDrive.getPower() * (averagepos / rightuppos));
            }*/
            telemetry.addData("inner while", leftDownDrive.getCurrentPosition());
            waitForNextHardwareCycle();
        }
        leftUpDrive.setPower(0.0);
        leftDownDrive.setPower(0.0);
        rightUpDrive.setPower(0.0);
        rightDowndrive.setPower(0.0);
        telemetry.addData("drive end",idist);

//        leftDownDrive.setMode(DcMotorController.RunMode.RESET_ENCODERS);
//        while(leftDownDrive.getCurrentPosition() != 0){
//            waitOneFullHardwareCycle();
//        }

    }

    public void bootinpress (){
        if(("Red".equals(teamColor)&& colorSensor.red()>1)||
                ("Blue".equals(teamColor)&& colorSensor.blue()>1)) {
            buttonpusher.setPosition(1.0);
            //pressbutton
        }else if (("Red".equals(teamColor)&& colorSensor.blue()>1)||
                ("Blue".equals(teamColor)&& colorSensor.red()>1)) {
            buttonpusher.setPosition(0.0);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        leftUpDrive = hardwareMap.dcMotor.get("lud");
        leftDownDrive = hardwareMap.dcMotor.get("ldd");
        rightDowndrive = hardwareMap.dcMotor.get("rdd");
        rightUpDrive = hardwareMap.dcMotor.get("rud");
        leftDownDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDowndrive.setDirection(DcMotor.Direction.REVERSE);
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        climber = hardwareMap.servo.get("climber");
        buttonpusher = hardwareMap.servo.get("bp");
        colorSensor = hardwareMap.colorSensor.get("color");
        colorSensor.enableLed(false);

        //    gyroCalibrate = sensorGyro.getRotation();
        while (menuChoice <= 8 && !opModeIsActive()) {
            telemetry.clearData();
            if (menuChoice == 1) {
                telemetry.addData("=> allience", teamColor);
            } else {
                telemetry.addData("   allience", teamColor);
            }

            if (menuChoice == 2) {
                telemetry.addData("=> start pos", startPositions[startPosition]);
            } else {
                telemetry.addData("   start pos", startPositions[startPosition]);
            }

            if (menuChoice == 3) {
                telemetry.addData("=> wait start", waitAtStart);
            } else {
                telemetry.addData("   wait start", waitAtStart);
            }

            if (menuChoice == 4) {
                telemetry.addData("=> bootin press", String.valueOf(pressButton));
            } else {
                telemetry.addData("   bootin press", String.valueOf(pressButton));
            }

            if (menuChoice == 5) {
                telemetry.addData("=> wait bootin", waitAtButton);
            } else {
                telemetry.addData("   wait bootin", waitAtButton);
            }

            if (menuChoice == 6) {
                telemetry.addData("=> block other bootin", String.valueOf(blockOtherButton));
            } else {
                telemetry.addData("   block other bootin", String.valueOf(blockOtherButton));
            }

            if (menuChoice == 7) {
                telemetry.addData("=> up mountain", upMountains[upMountain]);
            } else {
                telemetry.addData("   up mountain", upMountains[upMountain]);
            }

            if (menuChoice == 8) {
                telemetry.addData("=> climbers", String.valueOf(mountainClimbers));
            } else {
                telemetry.addData("   climbers", String.valueOf(mountainClimbers));
            }

            if (gamepad1.start && !startPress) {
                menuChoice = menuChoice + 1;
                startPress = true;
            } else if (!gamepad1.start) {
                startPress = false;
            }

            if (gamepad1.back && !backPress) {
                menuChoice = menuChoice - 1;
                backPress = true;
            } else if (!gamepad1.back) {
                backPress = false;
            }

            if (gamepad1.y && !yPress) {
                if (menuChoice == 1){
                    teamColor = "blue";
                } else if (menuChoice == 2) {
                    if (startPosition < startPositions.length) {
                        startPosition = startPosition + 1;
                    } else {
                        startPosition = 0;
                    }
                } else if (menuChoice == 3) {
                    if (waitAtStart < 20) {
                        waitAtStart = waitAtStart + 1;
                    } else {
                        waitAtStart = 0;
                    }
                } else if (menuChoice == 4) {
                    pressButton = true;
                } else if (menuChoice == 5) {
                    if (waitAtButton < 20){
                        waitAtButton = waitAtButton + 1;
                    } else {
                        waitAtButton = 0;
                    }
                } else if (menuChoice == 6) {
                    blockOtherButton = true;
                } else if (menuChoice == 7) {
                    if (upMountain < upMountains.length) {
                        upMountain = upMountain + 1;
                    } else {
                        upMountain = 0;
                    }
                } else  if (menuChoice == 8) {
                    mountainClimbers = true;
                }

                yPress = true;
            } else if (!gamepad1.y) {
                yPress = false;
            }

            if (gamepad1.a && !aPress) {
                if (menuChoice == 1) {
                    teamColor = "red";
                } else if (menuChoice == 2) {
                    if (startPosition > 0) {
                        startPosition = startPosition - 1;
                    } else {
                        startPosition = startPositions.length - 1;
                    }
                } else if (menuChoice == 3) {
                    if (waitAtStart > 0) {
                        waitAtStart = waitAtStart - 1;
                    } else {
                        waitAtStart = 20;
                    }
                } else if (menuChoice == 4) {
                    pressButton = false;
                } else if (menuChoice == 5) {
                    if (waitAtButton > 0){
                        waitAtButton = waitAtButton - 1;
                    } else {
                        waitAtButton = 20;
                    }
                } else if (menuChoice == 6) {
                    blockOtherButton = false;
                } else if (menuChoice == 7) {
                    if (upMountain > 0) {
                        upMountain = upMountain - 1;
                    } else {
                        upMountain = upMountains.length - 1;
                    }
                } else  if (menuChoice == 8) {
                    mountainClimbers = false;
                }

                aPress = true;
            } else if (!gamepad1.a) {
                aPress = false;
            }
        }

        sensorGyro.calibrate();
        //sleep(1000);
        //telemetry.addData("bias", gyroCalibrate);
//This is where we put and more init stuff like keeping the bot in 18x18
        //climber.setPosition(0.00);

        waitForStart();
        waitForNextHardwareCycle();
        //spinTurn(1.0,90);
        //sleep(2000);
        //spinTurn(-1.0,90);
        //1 move forward
        drive(0.75, 26);
        waitForNextHardwareCycle();
        sleep (1000);
        //2 turn right
        spinTurn(-1.0, 43);
        waitForNextHardwareCycle();
        //spinTurn(-1.0, 1);
        sleep(1000);
        //3 move forward
        drive(0.75, 95);
        waitForNextHardwareCycle();
        sleep (1000);
        //4 turn right
        spinTurn(-1.0, 35);
        waitForNextHardwareCycle();
        //5 move forward
        //drive(-.5, 24);
        bootinpress();
        //sleep(1000);
        //climber.setPosition(1.00);
        //6 move backward
        //drive(0.5, 30);
        //7 turn right
        //spinTurn(-.75, 90);
        //8 move forward
        //drive(-1.0, 30);
        //9 turn left
        //spinTurn(.5, 45);
        //10 move forward
        //drive(.5, 70);
        //sleep(1000);
    }
}
