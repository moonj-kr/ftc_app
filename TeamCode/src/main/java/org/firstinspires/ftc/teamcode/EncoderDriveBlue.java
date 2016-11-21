/**
 Updated: November 19, 2016
 First Competition Autonomous Red Side:
 - Drives using encoder methods
 - MR Optical Distance for white line
 - MR Color Sensors for beacon light detection

 Additional Notes: ANDYMARK_TICKS_PER_REV = 1120;
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "EncoderDriveBlue", group = "Linear Opmode")
@Disabled
public class EncoderDriveBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor M_drive_BL = null;
    DcMotor M_drive_BR = null;

    Servo S_button_FL = null;
    Servo S_button_FR = null;

    final double OPEN = 1.0;

    ColorSensor colorSensorLeft;

    OpticalDistanceSensor opticalDistanceSensor;

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
        //colorSensorRight = hardwareMap.colorSensor.get("color_FR");

        opticalDistanceSensor = hardwareMap.opticalDistanceSensor.get("ODS");

        //ColorSensor Values
        int redValue = colorSensorLeft.red();
        int greenValue = colorSensorLeft.green();
        int blueValue = colorSensorLeft.blue();

        //Motor Setup
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        M_drive_BL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        //ColorSensor state passive mode for beacons
        colorSensorLeft.enableLed(LEDState);

        //Drive
        DriveFowardDistance(.5, 3604); //end time:

        TurnLeft(.5,1875);            //end time:

        DriveFowardDistance(.5,-4170); //end time:

        TurnRight(.5,2480);             //end time:

        while(opticalDistanceSensor.getLightDetected() < .40){

            DriveFowardDistance(.2,-60);
        }

        while (redValue > blueValue && redValue > greenValue){

            S_button_FL.setPosition(1.0); //if red
            sleep(1500);
        }

        //UNTESTED CODE

        while (blueValue > redValue){
            DriveFowardDistance(.2,-1120); //to other beacon backwards
            S_button_FL.setPosition(1.0);
            sleep(1500);
        }

        //FROM FIRST TO SECOND BEACON

        DriveFowardDistance(.5,-8666);

        while(opticalDistanceSensor.getLightDetected() < .40){

            DriveFowardDistance(.2,-60);
        }

        while (redValue > blueValue && redValue > greenValue){

            S_button_FL.setPosition(1.0); //if red
            sleep(1500);
        }

        while (blueValue > redValue){
            DriveFowardDistance(.2,-1120); //to other beacon backwards
            S_button_FL.setPosition(1.0);
            sleep(1500);
        }

        //TURN 270 DEGREES
        TurnRight(.3,2813);

        //HITS CAPBALL
        DriveFowardDistance (.7, 8610);

    }


    public void DriveFowardDistance(double power, int distance) {
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

    public void StopDriving(double power, int distance) {
        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        sleep(1500);
        idle();
    }

    public void TurnRight(double power, int distance) {
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

    public void TurnLeft(double power, int distance) throws InterruptedException {
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
