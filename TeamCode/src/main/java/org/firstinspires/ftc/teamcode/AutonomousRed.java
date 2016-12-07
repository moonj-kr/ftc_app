
/**

 Second Competition Autonomous Red Side:
 - Drives using encoder methods
 - MR Optical Distance for white line
 - MR Color Sensors for beacon light detection
 -MR Range Sensor for distance from wall after turn

Additional Notes: ANDYMARK_TICKS_PER_REV = 1120;
*/

package org.firstinspires.ftc.teamcode;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous_Red", group = "Linear Opmode")

public class AutonomousRed extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
        DcMotor M_drive_BL,
                M_drive_BR,
                M_lift_FL,
                M_lift_FR,
                M_shooter;


        Servo S_button_FL,
              S_button_FR,
              S_liftSide_L,
              S_liftSide_R;

    final double OPEN = 1.0;

    ColorSensor colorSensorLeft;
    ColorSensor colorSensorRight; //different address

    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;

    ModernRoboticsI2cRangeSensor rangeSensorLeft;


    boolean LEDState = false;

    int TICKS_PER_REV = 1120; //one motor rotation
    int HALF_BLOCK = 861; // about 6 inches
    int ONE_BLOCK = 3444; // about 12 inches

    @Override
    public void runOpMode() throws InterruptedException {

        M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");

        S_button_FL = hardwareMap.servo.get("S_button_FL");
        S_button_FR = hardwareMap.servo.get("S_button_FR");

        colorSensorLeft = hardwareMap.colorSensor.get("color_FL");
        colorSensorRight = hardwareMap.colorSensor.get("color_FR");

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");

        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_FL");

        M_lift_FL = hardwareMap.dcMotor.get("M_lift_FL");
        M_lift_FR = hardwareMap.dcMotor.get("M_lift_FR");

        M_shooter = hardwareMap.dcMotor.get("M_shooter");

        S_liftSide_L = hardwareMap.servo.get("S_liftSide_L");
        S_liftSide_R = hardwareMap.servo.get("S_liftSide_R");

        //Motor Setup
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        M_drive_BL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //ColorSensor state passive mode for beacons
        colorSensorLeft.enableLed(LEDState);


        ///////////////Drive//////////////////
//TEST SERVO VALUES FOR EACH SIDE!

        S_button_FL.setPosition(0.0);
        S_button_FR.setPosition(0.0);
        S_button_FL.setPosition(1.0);
        sleep(1500);

        TurnRight(.5,937);                        //end time:
        //turn 45 degrees
        //1875/2 = 937

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
        */

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
        DriveFowardDistance (.7, 8610);
        //*/

    }


    public void DriveFowardDistance(double power, int distance) throws InterruptedException{
        //reset encoders
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        M_drive_BL.setTargetPosition(distance);
        M_drive_BR.setTargetPosition(distance);

        M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);

        //set to RUN_TO_POSITION mode
        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (M_drive_BL.isBusy() || M_drive_BR.isBusy()) {
            //wait until target position is reached
        }

        //stop and change modes back to normal
        //StopDriving(power, distance);
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();
    }

    public void DriveBackwardsDistance(double power, int distance) throws InterruptedException{
        //reset encoders
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        M_drive_BL.setTargetPosition(-distance);
        M_drive_BR.setTargetPosition(-distance);

        M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);

        //set to RUN_TO_POSITION mode
        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (M_drive_BL.isBusy() || M_drive_BR.isBusy()) {
            //wait until target position is reached
        }

        //stop and change modes back to normal
        //StopDriving(power, distance);
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        idle();
    }

    public void StopDriving(double power, int distance) {
        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        sleep(1500);
        idle();
    }

    public void TurnLeft(double power, int distance) throws InterruptedException{
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_drive_BR.setTargetPosition(distance);
        M_drive_BL.setTargetPosition(-distance);

        M_drive_BR.setPower(power);
        M_drive_BL.setPower(power);

        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_drive_BL.isBusy() || M_drive_BR.isBusy()) {
        }

        idle();
    }

    public void TurnRight(double power, int distance)throws InterruptedException{
        telemetry.addData("Encoder",M_drive_BR.getCurrentPosition());
        telemetry.addData("Encoder",M_drive_BL.getCurrentPosition());
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        M_drive_BR.setTargetPosition(-distance);
        M_drive_BL.setTargetPosition(distance);

        // M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);
        M_drive_BL.setPower(power);

        M_drive_BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_drive_BL.isBusy() || M_drive_BR.isBusy()) {

        }

        idle();

    }

    }
