
/**
 * Created by Jisook Moon on 1/5/17
 * State Autonomous for RED side
 * /

 Notes:
 - Drives using encoder methods
 - MR Optical Distance for white line
 - MR Color Sensors for beacon light detection
 -MR Range Sensor for distance from wall after turn

Additional Notes:
 ANDYMARK_TICKS_PER_REV = 1120;
 Original color sensor address: 0x3c
 Gyro:
 BNO055IMU imu;
 Orientation angles;
 Acceleration gravity;
*/


package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;
@Autonomous(name = "AutonomousRed", group = "Linear Opmode")
@Disabled
public class AutonomousRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    // motor declarations
    DcMotor M_drive_BL,
            M_drive_BR,
            M_drive_FL,
            M_drive_FR,
            M_lift_FL,
            M_lift_FR,
            M_shooter;

    //servo declarations
    Servo   S_button_FL,
            S_liftSide_L,
            S_liftSide_R,
            S_ballDrop;

    // sensor declarations
    ColorSensor colorSensorRight; //different address 0x3a
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;

    // color sensor constant
    boolean LEDState = false;

    final int first = 4004;
    final int oneAndHalfBlock = 5406;
    final int twoBlock = 7208;
    final int cornerToVortex = 8610;
    final int rightTurn = 2041;
    //1874+ 967 = 2841 right degree turn
    final int leftTurn = 937;

    final double ARM_INIT_POS_L = 0.8,
            ARM_INIT_POS_R = 0.235,
            BUTTON_INIT_POS = 0.8;


    @Override
    public void runOpMode() throws InterruptedException {

        // mapping motor variables to their hardware counter parts
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
        colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a));

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");

        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_FL");

        // motor encoder setup
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        M_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // fixing motor
        M_drive_BL.setDirection(DcMotor.Direction.REVERSE);
        M_drive_FL.setDirection(DcMotor.Direction.REVERSE);

        //this.S_button_FL.setPosition(BUTTON_INIT_POS);

        this.S_liftSide_L.setPosition(ARM_INIT_POS_L);
        this.S_liftSide_R.setPosition(ARM_INIT_POS_R);
        this.S_button_FL.setPosition(BUTTON_INIT_POS);
        this.S_ballDrop.setPosition(0.02);

        waitForStart();

        //ColorSensor state passive mode for beacons

        ///////////////Drive//////////////////

        //sleep(15000);

        DriveForwardDistance(0.5,2202);

        shooterRUN(0.5,-2340);

        S_ballDrop.setPosition(1.0);
        sleep(1500);

        S_ballDrop.setPosition(0.0);
        sleep(1500);

        shooterRUN(0.5,-2340);

        DriveForwardDistance(0.5,3903);

        /*


        S_ballDrop.setPosition(0.5);
        sleep(1500);
        shooterRUN(0.5,-2340);
        S_ballDrop.setPosition(0.0);
        sleep(1500);

        //shooterRUN(0.5,2340);
        //shooterRUN(0.5,-2340);


        /*

        DriveForwardDistance(.5,1502);

        shooterRUN(0.5, 1120); // runs shooter motor one rotation

        TurnRight(.5, 3736); //turns towards wall 45 degrees

        DriveBackwardsDistance(.5, 3004); // drives straight from wall

        TurnLeft(.3, 1801); // turns so that it is alligned against the wall

        while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected() < .40) {
            DriveForwardDistance(.2,-60);       // while the white line on the ground is not detected, run robot -60
        }


        if(colorSensorRight.red() > colorSensorRight.blue()){ //if beacon is red
            S_button_FL.setPosition(0.1); //extends servo
            sleep(1500);
            S_button_FL.setPosition(0.8); //reteract servo
            sleep(1500);
        }

        /*
        shooterRUN(0.5, 1120); // runs shooter motor one rotation

        DriveBackwardsDistance(.5, 3004); // drives straight from wall

        TurnRight(.5, 2491); //turns towards wall 90 degrees

        DriveForwardDistance(.5,4450); //drives towards wall after turn

        TurnLeft(.3, 1801); // turns so that it is alligned against the wall

        while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected() < .40) {
            DriveForwardDistance(.2,-60);       // while the white line on the ground is not detected, run robot -60
        }
        //after robot stops on white line

        if(colorSensorRight.red() > colorSensorRight.blue()){ //if beacon is red
            S_button_FL.setPosition(0.1); //extends servo
            sleep(1500);
            S_button_FL.setPosition(0.8); //reteract servo
            sleep(1500);
         }
        else if (colorSensorRight.blue() > colorSensorRight.red()){ //if beacon is blue
            DriveForwardDistance(.5,1214); //drive so it is in front of red
            S_button_FL.setPosition(0.1); //extend servo
            sleep(1500);
            S_button_FL.setPosition(0.8); //detract servo
            sleep(1500);
        }
        DriveForwardDistance(0.5,3104);

        /*


         */



/*

        TurnRight(.5,-rightTurn);
        DriveFowardDistance(.5,1402);
        TurnRight(.5,rightTurn);
        //DriveFowardDistance(.5,
        /*

        TurnRight(.5, leftTurn); //45 degrees

        while (rangeSensorLeft.getDistance(DistanceUnit.CM) > 8.0) {
            TurnLeft(.3, 10);
        }

        if (colorSensorLeft.red() > colorSensorLeft.blue()){
            S_button_FL.setPosition(0.1); //if red
            sleep(1500);
        }
        else {
            while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected() < .40) {
                DriveFowardDistance(.2,60);       //end time:
            }
        }

        if(colorSensorLeft.red() > colorSensorLeft.blue()){
            S_button_FL.setPosition(0.1);
        }




            //DriveFowardDistance(.5, );
        /*
        //turn 45 degrees //TURNS LEFT
        //1875/2 = 937
        DriveFowardDistance(.5,oneBlock);
        TurnLeft(.5, leftTurn);

        /*
        DriveFowardDistance(.5,oneAndHalfBlock);
        //hits wall






/*
        DriveFowardDistance(.5, 3604);           //end time:
        //drive forwards

        TurnRight(.5,2811);                      //end time:
        //turn 90+45 = 135
        //distance ~ 937*3 = 2811

        //check range (from wall parallel)
        while (rangeSensorLeft.getDistance(DistanceUnit.CM) > 8.0){
            TurnLeft(.3,10);
        }

        while(rangeSensorLeft.getDistance(DistanceUnit.CM) < 5.0){
            TurnRight(.3,10);
        }

        while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected() < .40) {

            DriveBackwardsDistance(.2,60);       //end time:
        }

        DriveBackwardsDistance(.5,2086);         //end time:
        //drive backwards
        //original -4170

        TurnLeft(.5,-2000);
        //turn 90 degrees
        //check gyro

        DriveFowardDistance(.5,1120);

        TurnRight(.5, 1875);
        //225 degrees
        //check gyro

        DriveFowardDistance(.5,-2086);

        TurnLeft(.5,2480);             //end time:
        //c

        while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected() < .40){

            DriveFowardDistance(.2,-60);
        }

        while (colorSensorLeft.red() > colorSensorLeft.blue()){

            S_button_FL.setPosition(1.0); //if red
            sleep(1500);
        }

        while (colorSensorLeft.blue() > colorSensorLeft.red()){
            DriveFowardDistance(.2,-1120); //to other beacon backwards
            S_button_FL.setPosition(1.0);
            sleep(1500);
        }

        //FROM FIRST TO SECOND BEACON

        DriveFowardDistance(.5,-8666);
/*
        while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected()< .40){

            DriveFowardDistance(.2,-60);
        }


        while (colorSensorLeft.red() > colorSensorLeft.blue()){

            S_button_FL.setPosition(1.0); //if red
            sleep(1500);
        }

        while (colorSensorLeft.blue() > colorSensorLeft.red()){
            DriveFowardDistance(.2,-1120); //to other beacon backwards
            S_button_FL.setPosition(1.0);
            sleep(1500);
        }

        //TURN 270 DEGREES
        TurnLeft(.3,2813);

        //HITS CAPBALL
        DriveFowardDistance (.7, cornerToVortex);
        //*/

        }


    //////////////METHODS//////////////////////////

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

    public void DriveForwardDistance(double power, int distance) throws InterruptedException{
        //reset encoders
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        M_drive_BL.setTargetPosition(distance);
        M_drive_BR.setTargetPosition(distance);
        M_drive_FL.setTargetPosition(distance);
        M_drive_FR.setTargetPosition(distance);

        M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);
        M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);

        //set to RUN_TO_POSITION mode
        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (M_drive_BL.isBusy() || M_drive_BR.isBusy() || M_drive_FL.isBusy() || M_drive_FR.isBusy()) {
            //wait until target position is reached
        }

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
    }

    public void DriveBackwardsDistance(double power, int distance) throws InterruptedException{
        //reset encoders
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        M_drive_BL.setTargetPosition(-distance);
        M_drive_BR.setTargetPosition(-distance);
        M_drive_FL.setTargetPosition(-distance);
        M_drive_FR.setTargetPosition(-distance);

        M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);
        M_drive_FL.setPower(power);
        M_drive_FR.setPower(power);     

        //set to RUN_TO_POSITION mode
        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (M_drive_BL.isBusy() || M_drive_BR.isBusy() || M_drive_FL.isBusy() || M_drive_FR.isBusy()) {
            //wait until target position is reached
        }

        //reset encoders
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
    }

    public void StopDriving(double power, int distance) {
        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        M_drive_FL.setPower(0);
        M_drive_FR.setPower(0);
        sleep(1500);
        idle();
    }

    public void TurnLeft(double power, int distance) throws InterruptedException{
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);     

        M_drive_BR.setTargetPosition(-distance);
        M_drive_BL.setTargetPosition(distance);
        M_drive_FR.setTargetPosition(-distance);
        M_drive_FL.setTargetPosition(distance);

        M_drive_BR.setPower(power);
        M_drive_BL.setPower(power);
        M_drive_FR.setPower(power);
        M_drive_FL.setPower(power);

        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_drive_BL.isBusy() || M_drive_BR.isBusy() || M_drive_FL.isBusy() || M_drive_FR.isBusy()) {
        }

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
    }

    public void TurnRight(double power, int distance)throws InterruptedException{

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        M_drive_BR.setTargetPosition(distance);
        M_drive_BL.setTargetPosition(-distance);
        M_drive_FR.setTargetPosition(distance);
        M_drive_FL.setTargetPosition(-distance);

        // M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);
        M_drive_BL.setPower(power);
        M_drive_FR.setPower(power);
        M_drive_FL.setPower(power);

        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_drive_BL.isBusy() || M_drive_BR.isBusy() || M_drive_FL.isBusy() || M_drive_FR.isBusy()) {

        }

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

    }}
