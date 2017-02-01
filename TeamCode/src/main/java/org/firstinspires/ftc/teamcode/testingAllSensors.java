package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name = "testingAllSensors", group = "Linear Opmode")

/**
 * Created by Jisook on 1/20/17.
 */


public class testingAllSensors extends LinearOpMode {

    // sensor declarations
    ColorSensor colorSensorRight;
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;
    ModernRoboticsI2cGyro gyroSensor;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;



    private void mapStuff() {

        // mapping sensor variables to their hardware counter parts

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");
        colorSensorRight = hardwareMap.colorSensor.get("color_FR");
        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_FL");
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        colorSensorRight.enableLed(false);

    }


    @Override
    public void runOpMode() throws InterruptedException {


        mapStuff();
        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;

        double case1Time = 0;
        boolean hasBeenSet = false;
        boolean finished = false;

        waitForStart();

        int tempMotorPosR = 0;
        int deltaMotorPos = 0;
        double increment = 0.05d;

        while(opModeIsActive()) {

            switch (counter) {
                case 0:

                    //use if opmode is active to continually check status

                    if(opticalDistanceSensor1.getLightDetected() < .4){
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.addData("HERE","HERE");
                        telemetry.update();
                        counter++;
                    }
                    break;
                case 1:
                    if(colorSensorRight.red() > colorSensorRight.blue()){
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.addData("RED","RED");
                        telemetry.update();
                        counter++;
                        }
                    break;
                case 2:
                    if(rangeSensorLeft.getDistance(DistanceUnit.CM) > 8.0){
                        telemetry.addData("GREATERTHAN88888", "RANGE");
                        telemetry.addData("GREATERTHAN88888", "RANGE");
                        telemetry.addData("GREATERTHAN88888", "RANGE");
                        telemetry.addData("GREATERTHAN88888", "RANGE");
                        telemetry.addData("GREATERTHAN88888", "RANGE");
                        telemetry.addData("GREATERTHAN88888", "RANGE");
                        telemetry.update();
                    }




/*
                        if (colorSensorRight.red() < colorSensorRight.blue()) {
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.update();
                        }
                        else if (colorSensorRight.red() > colorSensorRight.blue()) {
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");

                            telemetry.update();
                        }
                        else {
                            telemetry.addData("here", "CASE0");
                            telemetry.addData("here", "CASE0");
                            telemetry.addData("here", "CASE0");
                            telemetry.addData("here", "CASE0");
                            telemetry.addData("here", "CASE0");

                            telemetry.update();
                        }
                        counter++;

                    break;
                /*
                case 1:
                    if(!hasBeenSet) {
                        while (rangeSensorLeft.getDistance(DistanceUnit.CM) > 8.0) {
                            telemetry.addData("range HERE", "CASE1");
                            telemetry.addData("range HERE", "CASE1");
                            telemetry.addData("range HERE", "CASE1");
                            telemetry.addData("range HERE", "CASE1h");
                            telemetry.addData("range HERE", "CASE1");
                            telemetry.addData("range HERE", "CASE1");
                            telemetry.update();
                            counter ++;
                        }}
                        break;
                case 2:
                    if(!hasBeenSet){
                        curResetState = (gamepad1.a && gamepad1.b);
                        if(curResetState && !lastResetState)  {
                            gyroSensor.resetZAxisIntegrator();
                        }
                        lastResetState = curResetState;

                        // get the x, y, and z values (rate of change of angle).
                        xVal = gyroSensor.rawX();
                        yVal = gyroSensor.rawY();
                        zVal = gyroSensor.rawZ();


                        // get the heading info.
                        // the Modern Robotics' gyro sensor keeps
                        // track of the current heading for the Z axis only.
                        heading = gyroSensor.getHeading();
                        angleZ  = gyroSensor.getIntegratedZValue();

                        telemetry.addData(">", "Press A & B to reset Heading.");
                        telemetry.addData("0", "Heading %03d", heading);
                        telemetry.addData("1", "Int. Ang. %03d", angleZ);
                        telemetry.addData("2", "X av. %03d", xVal);
                        telemetry.addData("3", "Y av. %03d", yVal);
                        telemetry.addData("4", "Z av. %03d", zVal);
                        telemetry.update();


                        }
                        break;
                        */
                default:
                    telemetry.addData("default","default");
                    telemetry.addData("default","default");
                    telemetry.addData("default","default");
                    telemetry.addData("default","default");
                    telemetry.addData("default","default");
                    telemetry.addData("default","default");
                    telemetry.addData("default","default");

                    telemetry.update();
                    break;
            }

                    }}}
