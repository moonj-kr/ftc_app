/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;

/**
 * {} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */
@Autonomous(name = "SensorAdafruitIMUTest", group = "Sensor")

public class SensorAdafruitIMUTest extends LinearOpMode {
    //----------------------------------------------------------------------------------------------
    // State
    //----------------------------------------------------------------------------------------------

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    //motors
    DcMotor M_drive_BL,
            M_drive_BR;
    //----------------------------------------------------------------------------------------------
    // Main logic - PROGRAM HERE
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode () throws InterruptedException{

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

        //initialize motors
        M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");

        //Motor Setup
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BL.setDirection(DcMotor.Direction.REVERSE);

        // Wait until we're told to go
        waitForStart();

        /***READ INITIAL GYROSCOPE VALUES***/
        //read initial (double) gyro values to do calculations with
        double [] initValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String initVals = telemetrize();
        //display data to phone - can take this out later
        telemetry.addData("Data:", initVals);
        telemetry.update();

        /***TURN 90 DEGREES***/
        //Robot turns 90 degrees - have to find optimal values
        TurnRight(.5,937);

        /***READ FINAL GYROSCOPE VALUES***/
        //read (double) gyro values after turn to do calculations with
        double [] finalValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String finalVals = telemetrize();
        //display to phone - can take this out later
        telemetry.addData("Data after turning:", finalVals);
        telemetry.update();

        /***FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES***/
        double turnAngle = finalValsArray[0] - initValsArray[0];
        //convert double into string in order to display to phone
        String turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
        //display to phone
        telemetry.addData("Turn Angle: ", turnAngleString);
        telemetry.update();

        //WILL PROBABLY HAVE TO ADD A LARGER ROOM FOR ERROR HERE (85-95 deg)
        /***MAKE SURE TURN IS 90 DEGREES***/
        while (turnAngle != 90)
        {
            while (turnAngle < 90) {
                TurnRight(.5, 10);  //move wheels to compensate for turn that does not equal 90 deg

                /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG***/
                //read new compensated position
                finalValsArray = getAngles();
                //calculate difference from initial value AGAIN
                turnAngle = finalValsArray[0] - initValsArray[0];
                //convert double into string in order to display to phone
                turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                //display to phone
                telemetry.addData("Current ", turnAngleString);
                telemetry.update();
            }
            while (turnAngle > 90) {
                TurnLeft(.5, 10); //move wheels to compensate for turn that does not equal 90 deg

                /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG***/
                //read new compensated position
                finalValsArray = getAngles();
                //calculate difference from initial value AGAIN
                turnAngle = finalValsArray[0] - initValsArray[0];
                //convert double into string in order to display to phone
                turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                //display to phone
                telemetry.addData("Current ", turnAngleString);
                telemetry.update();
            }
        }
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
        double roll = Math.atan2( 2*(w*x +y*z), 1 - 2*(x*x + y*y)) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;

        return new double[]{yaw, pitch, roll};
    }

    // This method returns a string that can be used to output telemetry data easily in other classes.
    public String telemetrize() {
        double[] angles = getAngles();
        return String.format(Locale.US, "Yaw: %.3f  Pitch: %.3f  Roll: %.3f", angles[0], angles[1], angles[2]);
    }

    // WHEELS//
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

    public void gyroAngle(double [] initValsArray){
        //get double gyro values after turn to do calculations with
        double [] finalValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String finalVals = telemetrize();
        telemetry.addData("Data after turning:", finalVals);
        telemetry.update(); //might be incorrect

        //add difference of final and initial to telemetry
        double turnAngle = finalValsArray[0] - initValsArray[0];
        String turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
        telemetry.addData("Current ", turnAngleString);



    }
}

 /*
        //get double gyro values after turn to do calculations with
        double [] finalValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String finalVals = telemetrize();
        telemetry.addData("Data after turning:", finalVals);
        telemetry.update(); //might be incorrect


        //add difference of final and initial to telemetry
        double turnAngle = finalValsArray[0] - initValsArray[0];
        String turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
        telemetry.addData("Current ", turnAngleString);

        //might have to switch if and while statements
        //if the turn is not 90 degrees based on gyro values, turn the wheels until the turn is 90 degrees
        if (turnAngle != 90) {
            while (turnAngle < 90) {
                TurnRight(.5, 937);
            }
            while (turnAngle > 90) {
                TurnLeft(.5, 937);
            }
*/