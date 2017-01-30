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



    ElapsedTime clock;

    private void mapStuff() {

        // mapping sensor variables to their hardware counter parts
        colorSensorRight = hardwareMap.colorSensor.get("color_FR");

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1");
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");

        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_FL");
        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

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
        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        mapStuff();
        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        double case1Time = 0;
        boolean hasBeenSet = false;
        boolean finished = false;

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyroSensor.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyroSensor.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();
        clock.startTime();
        int tempMotorPosR = 0;
        int deltaMotorPos = 0;
        double increment = 0.05d;
        while(opModeIsActive()) {
            colorSensorRight.enableLed(false);


            switch (counter) {

                case 0:
                    if(!hasBeenSet) {
                        while (colorSensorRight.red() < colorSensorRight.blue()) {
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.addData("blue", "blue");
                            telemetry.update();
                        }
                        while (colorSensorRight.red() > colorSensorRight.blue()) {
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");
                            telemetry.addData("red", "red");

                            telemetry.update();
                        }
                        telemetry.addData("here", "CASE0");
                        telemetry.addData("here", "CASE0");
                        telemetry.addData("here", "CASE0");
                        telemetry.addData("here", "CASE0");
                        telemetry.addData("here", "CASE0");

                        telemetry.update();
                        counter++;
                    }
                    break;
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

                    }}}}