package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Arrays;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import java.util.Locale;

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
    Servo S_button_FL = null,  //button presser servo
            S_liftSide_L = null, //left lift release servo
            S_liftSide_R = null, //right lift release servo
            S_ballDrop = null;   // second ball release servo

    // sensor declarations
    ColorSensor colorSensorRight; //different address 0x3a
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;

    // The IMU sensor object
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    // all of the important constants
    final double ARM_INIT_POS_L = 0.8d,
            ARM_INIT_POS_R = 0.235d,
            BUTTON_INIT_POS = 0.8d;


    final double STOP = 0.0d,
            MAX_POWER = 1.0d;
    final int TICKS_PER_REVOLUTION = 1120;

    final double TurnRight45 = 45.0d,
            TurnLeft45 = -45.0d;


    // motor powers
    double M_drivePowerR = STOP,
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


        //IMU Mapping Hardware
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

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
        if (gamepad1.a) {
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
        while (opModeIsActive()) {
            colorSensorRight.enableLed(false);
            //double[]  initValsArray;
            switch (counter) {

                /////////ACTUAL TESED AUTONOMOUS PROGRAM/////////////////////////


                case 0:
                    //shooter run
                    /*
                    if (!hasBeenSet) {
                        shooterRUN(0.5, -2200); // previous -2160
                        shooterRUN(0.0, 0);
                        S_ballDrop.setPosition(1.0);
                        sleep(700); //previous 1500
                        S_ballDrop.setPosition(0.0);
                        sleep(700); //previous 1500
                        shooterRUN(0.5, -2200); //previous -2160
                        shooterRUN(0.0, 0);
                        hasBeenSet = true;
                        clock.reset();
                    }*/
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
                    //read initial (double) gyro values to do calculations with
                    double[] initValsArray = getAngles();
                    //store gyro values as string in order to display to phone
                    String initVals = telemetrize();
                    //display data to phone - can take this out later
                    telemetry.addData("Data:", initVals);
                    telemetry.update();
                    counter++;
                    break;

                case 3:
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

                case 4:
                    //gyro test
                    double[] initValsArray1 = {0, 0, 0};
                    boolean b = true;
                    double deg = 45;

                    //READ FINAL GYROSCOPE VALUES
                    //read (double) gyro values after turn to do calculations with
                    double[] finalValsArray = getAngles();
                    //store gyro values as string in order to display to phone
                    String finalVals = telemetrize();
                    //display to phone - can take this out later
                    telemetry.addData("Data after turning:", finalVals);
                    telemetry.update();

                    //FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES
                    //double turnAngle = turnAngle(finalValsArray, initValsArray1);
                    double turnAngle = finalValsArray[0] - initValsArray1[0];
                    turnAngle = Math.abs(turnAngle);
                    //convert double into string in order to display to phone
                    String turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                    //display to phone
                    telemetry.addData("Turn Angle: ", turnAngleString);
                    telemetry.update();

                    while (turnAngle > deg + 2 || turnAngle < deg - 2) { //2/1/17 - changed from IF to WHILE
                        telemetry.addData("here", "here");

                        //LEFT
                        if (turnAngle < deg-2) {

                            M_drive_FR.setPower(0.5d);
                            M_drive_FL.setPower(-0.5d);
                            M_drive_BR.setPower(0.5d);
                            M_drive_BL.setPower(-0.5d);

                            sleep(50);

                            M_drive_FR.setPower(0.0d);
                            M_drive_FL.setPower(0.0d);
                            M_drive_BR.setPower(0.0d);
                            M_drive_BL.setPower(0.0d);
                            //CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU
                            //read (double) gyro values after turn to do calculations with
                            finalValsArray = getAngles();
                            //store gyro values as string in order to display to phone
                            finalVals = telemetrize();
                            //display to phone - can take this out later
                            telemetry.addData("Data after turning:", finalVals);
                            telemetry.update();

                            //calculate difference from initial value AGAIN
                            //turnAngle = turnAngle(finalValsArray, initValsArray1);
                            turnAngle = finalValsArray[0] - initValsArray1[0];
                            turnAngle = Math.abs(turnAngle);
                            //convert double into string in order to display to phone
                            turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                            //display to phone
                            telemetry.addData("Turn Angle: ", turnAngleString);
                            telemetry.update();
                        }

                        //RIGHT
                        if (turnAngle > deg+2) {

                            M_drive_FR.setPower(-0.5d);
                            M_drive_FL.setPower(0.5d);
                            M_drive_BR.setPower(-0.5d);
                            M_drive_BL.setPower(0.5d);

                            sleep(50);

                            M_drive_FR.setPower(0.0d);
                            M_drive_FL.setPower(0.0d);
                            M_drive_BR.setPower(0.0d);
                            M_drive_BL.setPower(0.0d);
                            //CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU
                            //finalValsArray = getAngles();
                            //calculate difference from initial value AGAIN
                            //turnAngle = turnAngle(finalValsArray, initValsArray1);

                            //CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU
                            //read (double) gyro values after turn to do calculations with
                            finalValsArray = getAngles();
                            //store gyro values as string in order to display to phone
                            finalVals = telemetrize();
                            //display to phone - can take this out later
                            telemetry.addData("Data after turning:", finalVals);
                            telemetry.update();

                            //calculate difference from initial value AGAIN
                            //turnAngle = turnAngle(finalValsArray, initValsArray1);
                            turnAngle = finalValsArray[0] - initValsArray1[0];
                            turnAngle = Math.abs(turnAngle);
                            //convert double into string in order to display to phone
                            turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                            //display to phone
                            telemetry.addData("Turn Angle: ", turnAngleString);
                            telemetry.update();
                        }
                    }
//                    M_drive_FR.setPower(0.0d);
//                    M_drive_FL.setPower(0.0d);
//                    M_drive_BR.setPower(0.0d);
//                    M_drive_BL.setPower(0.0d);
                    hasBeenSet = false;

                    counter++;
                    break;

                case 5:
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

                case 6:
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-190.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 4", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 7:
                    //drives towards wall from turn
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-2.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 4", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 8:
                    //read initial (double) gyro values to do calculations with
                    initValsArray = getAngles();
                    //store gyro values as string in order to display to phone
                    initVals = telemetrize();
                    //display data to phone - can take this out later
                    telemetry.addData("Data:", initVals);
                    telemetry.update();
                    counter++;
                    break;

                case 9:
                    //turn 45 towards wall so is aligned
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-215.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 5", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }

                case 10:
                    //turn 45 towards wall so is aligned
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-215.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 5", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 11:
                    //turn 45 towards wall so is aligned
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-250.0d); //215
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 5", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 12:
                    //gyro test
                    double [] initValsArray2 = {0, 0, 0};
                    b = true;
                    double deg1 = 175; //180

                    //READ FINAL GYROSCOPE VALUES
                    //read (double) gyro values after turn to do calculations with
                    finalValsArray = getAngles();

                    //store gyro values as string in order to display to phone
                    finalVals = telemetrize();

                    //display to phone - can take this out later
                    telemetry.addData("Data after turning:", finalVals);
                    telemetry.update();

                    //FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES
                    turnAngle = finalValsArray[0] - initValsArray2[0];
                    turnAngle = Math.abs(turnAngle);

                    //convert double into string in order to display to phone
                    turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);

                    //display to phone
                    telemetry.addData("Turn Angle: ", turnAngleString);
                    telemetry.update();

                    while (turnAngle > deg1 + 2 || turnAngle < deg1 - 2) { //2/1/17 - changed from IF to WHILE
                        telemetry.addData("here", "here");

                        //LEFT
                        if (turnAngle < deg1 -2) {

                            M_drive_FR.setPower(-0.5d);
                            M_drive_FL.setPower(0.5d);
                            M_drive_BR.setPower(-0.5d);
                            M_drive_BL.setPower(0.5d);

                            sleep(50);

                            M_drive_FR.setPower(0.0d);
                            M_drive_FL.setPower(0.0d);
                            M_drive_BR.setPower(0.0d);
                            M_drive_BL.setPower(0.0d);
                            //CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU
                            //read (double) gyro values after turn to do calculations with
                            finalValsArray = getAngles();
                            //store gyro values as string in order to display to phone
                            finalVals = telemetrize();
                            //display to phone - can take this out later
                            telemetry.addData("Data after turning:", finalVals);
                            telemetry.update();

                            //calculate difference from initial value AGAIN
                            //turnAngle = turnAngle(finalValsArray, initValsArray1);
                            turnAngle = finalValsArray[0] - initValsArray2[0];
                            turnAngle = Math.abs(turnAngle);
                            //convert double into string in order to display to phone
                            turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                            //display to phone
                            telemetry.addData("Turn Angle: ", turnAngleString);
                            telemetry.update();
                        }

                        //RIGHT
                        if (turnAngle > deg1+2) {

                            M_drive_FR.setPower(0.5d);
                            M_drive_FL.setPower(-0.5d);
                            M_drive_BR.setPower(0.5d);
                            M_drive_BL.setPower(-0.5d);

                            sleep(50);

                            M_drive_FR.setPower(0.0d);
                            M_drive_FL.setPower(0.0d);
                            M_drive_BR.setPower(0.0d);
                            M_drive_BL.setPower(0.0d);
                            //CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU
                            //finalValsArray = getAngles();
                            //calculate difference from initial value AGAIN
                            //turnAngle = turnAngle(finalValsArray, initValsArray1);

                            //CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU
                            //read (double) gyro values after turn to do calculations with
                            finalValsArray = getAngles();
                            //store gyro values as string in order to display to phone
                            finalVals = telemetrize();
                            //display to phone - can take this out later
                            telemetry.addData("Data after turning:", finalVals);
                            telemetry.update();

                            //calculate difference from initial value AGAIN
                            //turnAngle = turnAngle(finalValsArray, initValsArray1);
                            turnAngle = finalValsArray[0] - initValsArray2[0];
                            turnAngle = Math.abs(turnAngle);
                            //convert double into string in order to display to phone
                            turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                            //display to phone
                            telemetry.addData("Turn Angle: ", turnAngleString);
                            telemetry.update();
                        }
                        telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", "hi");
                        telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", "hi");

                        telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", "hi");
                        telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", "hi");
                        telemetry.update();
                    }
//                    M_drive_FR.setPower(0.0d);
//                    M_drive_FL.setPower(0.0d);
//                    M_drive_BR.setPower(0.0d);
//                    M_drive_BL.setPower(0.0d);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", "hi");
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", "hi");
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.addData("HHHHHHHHHIIIIIIIIIIIIIIIIIIIIi Angle: ", turnAngleString);
                    telemetry.update();

                    hasBeenSet = false;

                    counter++;
                    break;

                case 13:
                    //drive forwards towards corner vortex

                    M_drive_FR.setPower(0.2d);
                    M_drive_FL.setPower(0.2d);
                    M_drive_BR.setPower(0.2d);
                    M_drive_BL.setPower(0.2d);
                    sleep(1000);

                    M_drive_FR.setPower(0.0d);
                    M_drive_FL.setPower(0.0d);
                    M_drive_BR.setPower(0.0d);
                    M_drive_BL.setPower(0.0d);

                    telemetry.update();
                    counter++;
                    break;

                case 14:
                    if (colorSensorRight.red() < colorSensorRight.blue()) {
                        telemetry.addData("blue", "itisnotred");
                        telemetry.addData("blue", "itisnotred");
                        telemetry.addData("blue", "itisnotred");
                        telemetry.addData("blue", "itisnotred");
                        telemetry.addData("blue", "itisnotred");
                        telemetry.update();
                        sleep(100);

                        if (!hasBeenSet) {
                            motorTargetsDrive = setDriveTarget(5.5d);
                            hasBeenSet = true;
                            clock.reset();
                        }

                        finished = driveForward();

                        if (finished || isPastTime(1.0d)) {
                            hasBeenSet = false;
                            stopDriving();
                            telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                            telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                            telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                            telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                            sleep(100);
                        }

                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //reteract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                        counter++;
                    }

                    if (colorSensorRight.red() > colorSensorRight.blue()){
                            telemetry.addData("red", "it is red");
                            telemetry.addData("red", "it is red");
                            telemetry.addData("red", "it is red");
                            telemetry.addData("red", "it is red");
                            telemetry.addData("red", "it is red");
                            telemetry.update();
                            sleep(100);

                            stopDriving();

                            S_button_FL.setPosition(0.1); //extends servo
                            sleep(1500);
                            S_button_FL.setPosition(0.8); //reteract servo
                            sleep(1500);
                            S_button_FL.setPosition(0.5); //stop servo
                            telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                            telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                            telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                            telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                            sleep(100);
                            counter++;
                        }

                    break;


                /*
                case 15:
                    //drives to second beacon
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-122.0d);
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

                case 16:
                    if(colorSensorRight.red() < colorSensorRight.blue()) {
                        telemetry.addData("blue","itisnotred");
                        telemetry.update();

                        //drive forwards to second button
                        if (!hasBeenSet) {
                            motorTargetsDrive = setDriveTarget(-5.5d);
                            hasBeenSet = true;
                            clock.reset();
                        }

                        finished = driveForward();
                        if (finished || isPastTime(1.0d)) {
                            hasBeenSet = false;
                            stopDriving();
                            telemetry.addData("RF POS", M_drive_FR.getCurrentPosition());
                            telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                            telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                            telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                            sleep(100);
                        }

                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //retract servo
                        sleep(1500);
                        S_button_FL.setPosition(0.5); //stop servo
                        counter++;
                    }

                    else {
                        telemetry.addData("red","it is red");
                        telemetry.update();
                        stopDriving();
                        S_button_FL.setPosition(0.1); //extends servo
                        sleep(1500);
                        S_button_FL.setPosition(0.8); //retract servo
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

                    */

                case 17:
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(55.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 5", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
/*
                case 18:
                    if (!hasBeenSet) {
                        motorTargetsTurn = setTurnTarget(-215.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = turnRight();
                    if (finished || isPastTime(0.6d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS 5", M_drive_FR.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_FL.getCurrentPosition());
                        telemetry.addData("RB POS", M_drive_BR.getCurrentPosition());
                        telemetry.addData("LB POS", M_drive_BL.getCurrentPosition());
                        sleep(100);
                    }
*/
                case 18:
                    //drive forwards towards corner vortex
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-90.0d);
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

            M_drive_FR.setPower(M_drivePowerR);
            M_drive_FL.setPower(M_drivePowerL);
            M_drive_BR.setPower(M_drivePowerR);
            M_drive_BL.setPower(M_drivePowerL);

            telemetry.addData("Counter", counter);
            telemetry.addData("Supposed Power R", M_drivePowerR);
            telemetry.addData("Supposed Power L", M_drivePowerL);
            telemetry.addData("time", clock.time());
            sleep(20);
        }
    }

    public boolean isPastTime(double maxTime) {
        if (clock.time() > maxTime) {
            return true;
        }
        else {
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

    public void shooterRUN(double power, int distance) throws InterruptedException {
        M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_shooter.setTargetPosition(distance);

        M_shooter.setPower(power);

        M_shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_shooter.isBusy()) {
            //wait
        }


        idle();
    }


    public int[] setDriveTarget(double inches) {
        final double INCHES_TO_TICKS = 1100.0d / 12.0d;
        DcMotor[] motors = {M_drive_FR, M_drive_FL, M_drive_BR, M_drive_BR};
        int[] targets = new int[2];
        targets[0] = (int) (motors[0].getCurrentPosition() + inches * INCHES_TO_TICKS);
        targets[1] = (int) (motors[1].getCurrentPosition() + inches * INCHES_TO_TICKS);
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
                if (i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                /*motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);*/
                if (i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
                //drivePowers[i] = 0.0d;
            }
        }
        if (Math.abs((M_drive_FR.getCurrentPosition() + M_drive_BR.getCurrentPosition()) / 2.0d - motorTargetsDrive[0]) > 30) {
            return false;
        } else if (Math.abs((M_drive_FL.getCurrentPosition() + M_drive_BL.getCurrentPosition()) / 2.0d - motorTargetsDrive[1]) > 30) {
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
        targets[0] = (int) (motors[0].getCurrentPosition() - degrees * DEGREES_TO_TICKS);
        targets[1] = (int) (motors[1].getCurrentPosition() + degrees * DEGREES_TO_TICKS);
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
                if (i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                /*motors[i].setPower(0.0d);
                motors[i + 2].setPower(0.0d);*/
                if (i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
                //drivePowers[i] = 0.0d;
            }
        }
        if (Math.abs((M_drive_FR.getCurrentPosition() + M_drive_BR.getCurrentPosition()) / 2.0d - motorTargetsTurn[0]) > 30) {
            return false;
        } else if (Math.abs((M_drive_FL.getCurrentPosition() + M_drive_BL.getCurrentPosition()) / 2.0d - motorTargetsTurn[1]) > 30) {
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

    /***************
     * IMU METHODS
     ***************/

    //----------------------------------------------------------------------------------------------
    // IMU Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

//----------------------------------------------------------------------------------------------
// Quaternion --> Euler Angles and Orientation of Sensor
    //Used this example
    //https://github.com/MasqedMarauder/FTC-Masq-Samples/blob/master/AdafruitIMU/MasqAdafruitIMU.java
//----------------------------------------------------------------------------------------------

    public double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();


        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;


        //for Adafruit IMU, yaw and roll are switched
        //equations from Wikipedia. Converts quaternion values to euler angles
        double roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)) * 180.0 / Math.PI;
        double pitch = Math.asin(2 * (w * y - x * z)) * 180.0 / Math.PI;
        double yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }

    public void perfectTurn(double deg, double[] initValsArray, boolean b, boolean hasBeenSet, boolean finished) throws InterruptedException {
        // while (b == true) {
        /***READ FINAL GYROSCOPE VALUES***/
        //read (double) gyro values after turn to do calculations with
        double[] finalValsArray = readFinalGyro();

        /***FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES***/
        double turnAngle = turnAngle(finalValsArray, initValsArray);

        //fix the turn to match X degrees
        fixTurn(deg, turnAngle, finalValsArray, initValsArray, b, hasBeenSet, finished);
        // }
    }

    // This method returns a string that can be used to output telemetry data easily in other classes.
    public String telemetrize() {
        double[] angles = getAngles();
        return String.format(Locale.US, "Yaw: %.3f  Pitch: %.3f  Roll: %.3f", angles[0], angles[1], angles[2]);
    }

    public double[] readInitialGyro() {
        //read initial (double) gyro values to do calculations with
        double[] initValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String initVals = telemetrize();
        //display data to phone - can take this out later
        telemetry.addData("Data:", initVals);
        telemetry.update();
        return initValsArray;
    }

    public double[] readFinalGyro() {
        //read (double) gyro values after turn to do calculations with
        double[] finalValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String finalVals = telemetrize();
        //display to phone - can take this out later
        telemetry.addData("Data after turning:", finalVals);
        telemetry.update();
        return finalValsArray;
    }

    public double turnAngle(double[] finalValsArray, double[] initValsArray) {
        /***FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES***/
        double turnAngle = finalValsArray[0] - initValsArray[0];
        turnAngle = Math.abs(turnAngle);
        //convert double into string in order to display to phone
        String turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
        //display to phone
        telemetry.addData("Turn Angle: ", turnAngleString);
        telemetry.update();
        return turnAngle;
    }

    public void fixTurn(double deg, double turnAngle, double[] finalValsArray, double[] initValsArray, boolean b, boolean hasBeenSet, boolean finished) throws InterruptedException {
        //WILL PROBABLY HAVE TO ADD A LARGER ROOM FOR ERROR HERE (85-95 deg)
        while (turnAngle > deg+5 || turnAngle < deg-5) { //2/1/17 - changed from IF to WHILE
            if (turnAngle < deg-5) {

                if (!hasBeenSet) {
                    motorTargetsTurn = setTurnTarget(-3.0d);
                    hasBeenSet = true;
                    clock.reset();
                }
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

                /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU***/
                finalValsArray = getAngles();

                //calculate difference from initial value AGAIN
                turnAngle = turnAngle(finalValsArray, initValsArray);
            }

            if (turnAngle > deg+5) {
                if (!hasBeenSet) {
                    motorTargetsTurn = setTurnTarget(3.0d);
                    hasBeenSet = true;
                    clock.reset();
                }
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

                /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU***/
                finalValsArray = getAngles();
                //calculate difference from initial value AGAIN
                turnAngle = turnAngle(finalValsArray, initValsArray);
            }
        }
    }
}





//drives towards wall from turn