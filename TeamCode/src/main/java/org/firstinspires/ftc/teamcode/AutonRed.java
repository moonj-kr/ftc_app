package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

@Autonomous(name = "AutonRed", group = "Linear Opmode")

/**
 * Created by Jisook on 11/29/17.
 */


public class AutonRed extends LinearOpMode {

    // motor declarations
    DcMotor M_drive_BL = null, // back left drive motors
            M_drive_BR = null, // back right drive motor
            M_drive_FL = null, //front left drive motor
            M_drive_FR = null, //front right drive motor
            M_lift_FL = null,  // front left lift motor
            M_lift_FR = null,  //front right lift motor
            M_shooter = null;  //shooter motor

    //servo declarations
    Servo   S_button_FL = null,  //button presser servo
            S_liftSide_L = null, //left lift release servo
            S_liftSide_R = null, //right lift release servo
            S_ballDrop = null;   // second ball release servo

    // sensor declarations
    ColorSensor colorSensorRight; //different address 0x3a
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;
    ModernRoboticsI2cGyro gyroSensor;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;

    // all of the important constants
    final double    ARM_INIT_POS_L = 0.8d,
                    ARM_INIT_POS_R = 0.235d,
                    BUTTON_INIT_POS = 0.8d;


    final double    STOP                   = 0.0d,
                    MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    final double TurnRight45 = 45.0d,
                 TurnLeft45 = -45.0d;


    // motor powers
    double  M_drivePowerR = STOP,
            M_drivePowerL = STOP;
    double[] drivePowers;

    // function necessity delcarations
    int[] motorTargetsDrive;
    int[] motorTargetsTurn;

    ElapsedTime clock;

    private void mapStuff() {
        // mapping motor variables to their hardware counterparts
        M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");
        M_drive_FL = hardwareMap.dcMotor.get("M_drive_FL");
        M_drive_FR = hardwareMap.dcMotor.get("M_drive_FR");
        M_lift_FL = hardwareMap.dcMotor.get("M_lift_FL");
        M_lift_FR = hardwareMap.dcMotor.get("M_lift_FR");
        M_shooter = hardwareMap.dcMotor.get("M_shooter");

        // mapping servo variables to their hardware counter parts
        S_liftSide_L = hardwareMap.servo.get("S_liftSide_L");
        S_liftSide_R = hardwareMap.servo.get("S_liftSide_R");
        S_button_FL = hardwareMap.servo.get("S_button_FL");
        S_ballDrop = hardwareMap.servo.get("S_ballDrop");

        // mapping sensor variables to their hardware counter parts
        colorSensorRight = hardwareMap.colorSensor.get("color_FR");
        //colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a));

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");


        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_FL");
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");



        // The IMU sensor object
        BNO055IMU imu;
        Orientation angles;
        Acceleration gravity;


    }

    private void configureStuff() {
        // fixing motor directions
        this.M_drive_FR.setDirection(DcMotor.Direction.REVERSE);
        this.M_drive_BR.setDirection(DcMotor.Direction.REVERSE);

        // resets all the encoder values
        this.M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        M_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.S_liftSide_L.setPosition(ARM_INIT_POS_L);
        this.S_liftSide_R.setPosition(ARM_INIT_POS_R);
        this.S_button_FL.setPosition(BUTTON_INIT_POS);
        this.S_ballDrop.setPosition(0.02);

        motorTargetsDrive = new int[2];
        motorTargetsTurn = new int[2];
        Arrays.fill(motorTargetsDrive, 0);
        Arrays.fill(motorTargetsTurn, 0);

        clock = new ElapsedTime();
    }

    private boolean waitingForClick() {
        telemetry.addData("Waiting for click", "waiting");
        if(gamepad1.a) {
            return false;
        }
        return true;
    }
    @Override
    public void runOpMode() throws InterruptedException {
        mapStuff();
        configureStuff();
        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        double case1Time = 0;
        boolean hasBeenSet = false;
        boolean finished = false;
        drivePowers = new double[2];
        Arrays.fill(drivePowers, 0.0d);
        waitForStart();
        clock.startTime();
        int tempMotorPosR = 0;
        int deltaMotorPos = 0;
        double increment = 0.05d;
        while(opModeIsActive()) {
            colorSensorRight.enableLed(false);

            switch (counter) {

            /*
                case 0:
                    while (!hasBeenSet) {
                        double currentDistance = rangeSensorLeft.getDistance(DistanceUnit.CM);
                        if (currentDistance > 7.0 || currentDistance < 9.0) {

                            telemetry.addData("Range", currentDistance);
                            telemetry.addData("Range", currentDistance);
                            telemetry.addData("Range", currentDistance);
                            telemetry.addData("Range", currentDistance);
                            telemetry.addData("Range", currentDistance);
                            telemetry.addData("Range", currentDistance);
                            telemetry.update();

                            motorTargetsDrive = setDriveTarget(10.0d);
                            clock.reset();
                        }
                        finished = driveForward();
                        if (finished || isPastTime(1.0d)) {
                            hasBeenSet = true;
                            counter++;
                            stopDriving();
                            telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                            telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                            telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                            telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                            sleep(100);
                        }

                        while (currentDistance <= 7.0){
                            if (!hasBeenSet) { //decide if this is needed
                                motorTargetsTurn = setTurnTarget(-2);
                                clock.reset();
                            }
                            hasBeenSet = true;
                            finished = turnRight();
                            if (finished || isPastTime(0.6d)) {
                                hasBeenSet = false;
                                stopDriving();
                                telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                                telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                                telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                                telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                                sleep(100);
                            }
                            break;
                        }

                        while (currentDistance >=9.0){
                            if (!hasBeenSet) { //decide if this is needed
                                motorTargetsTurn = setTurnTarget(2);
                                clock.reset();
                            }
                            hasBeenSet = true;
                            finished = turnRight();
                            if (finished || isPastTime(0.6d)) {
                                hasBeenSet = false;
                                stopDriving();
                                telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                                telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                                telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                                telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                                sleep(100);
                            }
                            break;
                        }
                    }
                    hasBeenSet = false;
                    break;



    /*
                case 0:

                        while (colorSensorRight.red() < colorSensorRight.blue()){
                            telemetry.addData("blue","blue");
                            telemetry.addData("blue","blue");
                            telemetry.addData("blue","blue");
                            telemetry.addData("blue","blue");
                            telemetry.addData("blue","blue");
                            telemetry.addData("blue","blue");
                            telemetry.update();
                        }
                        while(colorSensorRight.red() > colorSensorRight.blue()) {
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");

                            telemetry.update();
                        }
                    telemetry.addData("here","iam");
                    telemetry.addData("here","iam");
                    telemetry.addData("here","iam");
                    telemetry.addData("here","iam");
                    telemetry.addData("here","iam");

                    telemetry.update();
                    break;
                case 1:
                    if(!hasBeenSet){
                        while (rangeSensorLeft.getDistance(DistanceUnit.CM) > 8.0) {
                            telemetry.addData("range HERE", "rangeahhhhh");
                            telemetry.addData("range HERE", "rangeahhhhh");
                            telemetry.addData("range HERE", "rangeahhhhh");
                            telemetry.addData("range HERE", "rangeahhhhh");
                            telemetry.addData("range HERE", "rangeahhhhh");
                            telemetry.addData("range HERE", "rangeahhhhh");
                            telemetry.update();
                        }

                    }
*/
            /////////ACTUAL TESED AUTONOMOUS PROGRAM/////////////////////////
/*

            case 0:

                /*
                //shooter run
                if (!hasBeenSet) {
                    shooterRUN(0.5, -2160);
                    shooterRUN(0.0, 0);
                    S_ballDrop.setPosition(1.0);
                    sleep(1500);
                    S_ballDrop.setPosition(0.0);
                    sleep(1500);
                    shooterRUN(0.5, -2160);
                    shooterRUN(0.0, 0);
                    hasBeenSet = true;
                    clock.reset();
                }


                telemetry.addData("shooter","shooter");
                hasBeenSet = false;
                counter++;
                break;

                case 1:
                    //drive forwards towards corner vortex
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-20.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 2:
                    //turn -135 degrees from starting position
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-135.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 3:
                    //drives towards wall from turn
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-176.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;
                case 4:
                    //drives towards wall from turn
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-180.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                */
                case 0:
                    //turn 45 towards wall so is aligned
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-720.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;
/*
                case 5:
                    //drive forwards first beacon
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(60.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 6:
                    if(colorSensorRight.red() < colorSensorRight.blue()) {
                        telemetry.addData("blue","itisnotred");
                        telemetry.update();
                        M_drivePowerR = 0.2d;
                        M_drivePowerL = 0.2d;
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //retract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;

                    } else {
                        telemetry.addData("red","it is red");
                        telemetry.update();
                        stopDriving();
                        deltaMotorPos = M_drive_FR.getCurrentPosition() - tempMotorPosR;
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //reteract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 7:
                    //drives to second beacon
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-120.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 8:

                    if(colorSensorRight.red() < colorSensorRight.blue()) {
                        telemetry.addData("blue","itisnotred");
                        telemetry.update();
                        M_drivePowerR = 0.2d;
                        M_drivePowerL = 0.2d;
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //retract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;

                    } else {
                        telemetry.addData("red","it is red");
                        telemetry.update();
                        stopDriving();
                        deltaMotorPos = M_drive_FR.getCurrentPosition() - tempMotorPosR;
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //reteract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 9:


            /*
                case 1:
                    //turn 45 degrees from starting position
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(TurnRight45);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 2:
                    //drive forwards towards corner vortex
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(5.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 3:
                    //turn -45 aligned with corner vortex
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(TurnLeft45);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 4:
                    //drive towards button presser wall
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(19.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 5:
                    //turn -45 aligned with wall
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(TurnLeft45);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    //play around with range
                    while(rangeSensorLeft.getDistance(DistanceUnit.CM) > 8.0){
                        motorTargetsTurn = setTurnTarget(1.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 6:
                    if(colorSensorRight.red() < colorSensorRight.blue()) {
                        telemetry.addData("blue","itisnotred");
                        telemetry.update();
                        M_drivePowerR = 0.2d;
                        M_drivePowerL = 0.2d;
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //retract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;

                    } else {
                        telemetry.addData("red","it is red");
                        telemetry.update();
                        stopDriving();
                        deltaMotorPos = M_drive_FR.getCurrentPosition() - tempMotorPosR;
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //reteract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;
                            telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                            telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                            telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                            telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                            sleep(100);
                    }
                    break;

                case 7:
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(5.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;
                case 8:
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(90.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                     break;
                case 9:
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(25.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;
*/
                default:
                    M_drivePowerR = STOP;
                    M_drivePowerL = STOP;
                    this.M_shooter.setPower(STOP);
                    this.M_lift_FL.setPower(STOP);
                    this.M_lift_FR.setPower(STOP);
                    telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                    telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                    telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                    telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                    telemetry.addData("Target R", motorTargetsDrive[0]);
                    telemetry.addData("Target L", motorTargetsDrive[1]);
                    break;
            }
            //M_drivePowerR = drivePowers[0];
            //M_drivePowerL = drivePowers[1];
            M_drive_FR.setPower(M_drivePowerR);
            M_drive_FL.setPower(M_drivePowerL);
            M_drive_BR.setPower(M_drivePowerR);
            M_drive_BL.setPower(M_drivePowerL);


            telemetry.addData("Counter", counter);
            telemetry.addData("Supposed Power R", M_drivePowerR);
            telemetry.addData("Supposed Power L", M_drivePowerL);
            telemetry.addData("time", clock.time());
            //waitOneFullHardwareCycle();
            sleep(20);
        }}


    private boolean isRed() {   return colorSensorRight.red() > colorSensorRight.blue();  }

    public boolean isPastTime(double maxTime) {
        if(clock.time() > maxTime) {
            return true;
        } else {
            return false;
        }
    }

    public void stopDriving() {
        M_drivePowerR = STOP;
        M_drivePowerL = STOP;
        M_drive_FR.setPower(STOP);
        M_drive_FL.setPower(STOP);
        M_drive_BR.setPower(STOP);
        M_drive_BL.setPower(STOP);
    }


    public void shooterRUN (double power, int distance) throws InterruptedException{
        M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_shooter.setTargetPosition(distance);

        M_shooter.setPower(power);

        M_shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(M_shooter.isBusy()){
            //wait
        }



        idle();
    }


    public int[] setDriveTarget(double inches) {
        final double INCHES_TO_TICKS = 1100.0d / 12.0d;
        DcMotor[] motors = {M_drive_FR, M_drive_FL, M_drive_BR, M_drive_BR};
        int[] targets = new int[2];
        targets[0] = (int)(motors[0].getCurrentPosition() + inches * INCHES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + inches * INCHES_TO_TICKS);
        return targets;
    }

    // 12 goes 14.5

    public boolean driveForward() {
        DcMotor[] motors = {M_drive_FR, M_drive_FL, M_drive_BR, M_drive_BR};
        double[] PIDValue = new double[2];
        double[] accumError = new double[2];
        double kP = 0.002d;
        double kI;
        double[] actualPIDValue = new double[2];
        double thresholdPower = 0.1d;

        for (int i = 0; i < 2; i++) {
            int error = motorTargetsDrive[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            accumError[i] += error;
            actualPIDValue[i] = kP * error;
            if (Math.abs(actualPIDValue[i]) > thresholdPower) {
                /*motors[i].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));
                motors[i + 2].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));*/
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                /*motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);*/
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
                //drivePowers[i] = 0.0d;
            }
        }
        if(Math.abs((M_drive_FR.getCurrentPosition() + M_drive_BR.getCurrentPosition()) / 2.0d - motorTargetsDrive[0]) > 30) {
            return false;
        } else if(Math.abs((M_drive_FL.getCurrentPosition() + M_drive_BL.getCurrentPosition()) / 2.0d - motorTargetsDrive[1]) > 30) {
            return false;
        }
        return true;
    }

    public int[] setTurnTarget(double degrees) {
        final double DEGREES_TO_TICKS = 1160.0d / 90.0d;
        DcMotor[] motors = {M_drive_FR, M_drive_FL, M_drive_BR, M_drive_BR};
        int[] targets = new int[2];
        //targets[0] = (int)((motors[0].getCurrentPosition() + motors[2].getCurrentPosition()) / 2 - degrees * DEGREES_TO_TICKS);
        //targets[1] = (int)((motors[1].getCurrentPosition() + motors[3].getCurrentPosition()) / 2 + degrees * DEGREES_TO_TICKS);
        targets[0] = (int)(motors[0].getCurrentPosition() - degrees * DEGREES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + degrees * DEGREES_TO_TICKS);
        return targets;
    }

    public boolean turnRight() {
        DcMotor[] motors = {M_drive_FR, M_drive_FL, M_drive_BR, M_drive_BR};
        double[] PIDValue = new double[2];
        double[] accumError = new double[2];
        double kP = 0.002d;
        double kI;
        double actualPIDValue[] = new double[2];
        double thresholdPower = 0.1d;

        for (int i = 0; i < 2; i++) {
            //int error = (int)(motorTargetsTurn[i] - (motors[i].getCurrentPosition() - motors[i + 2].getCurrentPosition()) / 2);
            int error = motorTargetsTurn[i] - motors[i].getCurrentPosition();
            PIDValue[i] = kP * error;
            accumError[i] += error;
            actualPIDValue[i] = kP * error;
            if (Math.abs(actualPIDValue[i]) > 0.05) {
                /*motors[i].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));
                motors[i + 2].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));*/
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                /*motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);*/
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
                //drivePowers[i] = 0.0d;
            }
        }
        if(Math.abs((M_drive_FR.getCurrentPosition() + M_drive_BR.getCurrentPosition()) / 2.0d - motorTargetsTurn[0]) > 30) {
            return false;
        } else if(Math.abs((M_drive_FL.getCurrentPosition() + M_drive_BL.getCurrentPosition()) / 2.0d - motorTargetsTurn[1]) > 30) {
            return false;
        }
        return true;
    }

    /*private class DriveThread extends PID {
        public DriveThread(int gearRatio, int objectCircumference) {
            GEAR_RATIO = gearRatio;
            OBJECT_CIRCUMFERENCE = objectCircumference;
            kP = 0.0f;
            kI = 0.0f;
            kD = 0.0f;
            maxPower = 1.0f;
            minPower = -1.0f;
            minPIDPower = 0.2f;
            acceptableError = 50;
        }
        public void setTarget(float target) {
            this.target = target;
        }
    }
    */
}
