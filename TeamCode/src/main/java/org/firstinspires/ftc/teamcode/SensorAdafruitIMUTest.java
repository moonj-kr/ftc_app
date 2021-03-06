package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;

import java.util.Locale;

/**
* {@link SensorAdafruitIMUTest} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
*
* Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
* Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
*
* @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
*/
@Autonomous(name = "SensorAdafruitIMUTest", group = "Linear Opmode")
@Disabled
// Uncomment this to add to the opmode list
public class SensorAdafruitIMUTest extends LinearOpMode {
   //----------------------------------------------------------------------------------------------
   // State
   //----------------------------------------------------------------------------------------------

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

   // State used for updating telemetry
   Orientation angles;
   Acceleration gravity;
   
   // The IMU sensor object
   BNO055IMU imu;
   
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
        M_drive_BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        M_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // fixing motor
        M_drive_BL.setDirection(DcMotor.Direction.REVERSE);
        M_drive_FL.setDirection(DcMotor.Direction.REVERSE);

        //this.S_button_FL.setPosition(BUTTON_INIT_POS);

        this.S_liftSide_L.setPosition(ARM_INIT_POS_L);
        this.S_liftSide_R.setPosition(ARM_INIT_POS_R);
        this.S_button_FL.setPosition(BUTTON_INIT_POS);
        this.S_ballDrop.setPosition(0.02);

       //Gyro Mapping Hardware
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


       // Wait until we're told to go
       waitForStart();

/***READ INITIAL GYROSCOPE VALUES***/
       //read initial (double) gyro values to do calculations with
       double[] initValsArray = getAngles();
       //store gyro values as string in order to display to phone
       String initVals = telemetrize();
       //display data to phone - can take this out later
       telemetry.addData("Data:", initVals);
       telemetry.update();

       /*************turn 90******/
       TurnLeft(0.5,3000); //<90

       DriveForwardDistance(0.5,3000);

       while (opModeIsActive()) {

           /***READ FINAL GYROSCOPE VALUES***/
           //read (double) gyro values after turn to do calculations with
           double[] finalValsArray = getAngles();
           //store gyro values as string in order to display to phone
           String finalVals = telemetrize();
           //display to phone - can take this out later
           telemetry.addData("Data after turning:", finalVals);
           telemetry.update();

           /***FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES***/
           double turnAngle = finalValsArray[0] - initValsArray[0];
           turnAngle = Math.abs(turnAngle);
           //convert double into string in order to display to phone
           String turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
           //display to phone
           telemetry.addData("Turn Angle: ", turnAngleString);
           telemetry.update();

           //WILL PROBABLY HAVE TO ADD A LARGER ROOM FOR ERROR HERE (85-95 deg)
           /***MAKE SURE TURN IS 90 DEGREES***/
           //if ((gyro.getHeading() <= 90 + TOLERANCE) && (gyro.getHeading() >= 90 - TOLERANCE)) {

           if (turnAngle > 95 || turnAngle < 85) {

               if (turnAngle < 85) {
                   TurnLeft(.5, 50);  //move wheels to compensate for turn that does not equal 90 deg

                   /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG***/
                   //read new compensated position
                   finalValsArray = getAngles();
                   //calculate difference from initial value AGAIN
                   turnAngle = finalValsArray[0] - initValsArray[0];
                   turnAngle = Math.abs(turnAngle);
                   //convert double into string in order to display to phone
                   turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", Math.abs(turnAngle));
                   //display to phone
                   telemetry.addData("Current ", turnAngleString);
                   telemetry.update();
               }
               if (turnAngle > 95) {
                   TurnRight(.5, 50); //move wheels to compensate for turn that does not equal 90 deg

                   /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG***/
                   //read new compensated position
                   finalValsArray = getAngles();
                   //calculate difference from initial value AGAIN
                   turnAngle = finalValsArray[0] - initValsArray[0];
                   turnAngle = Math.abs(turnAngle);
                   //convert double into string in order to display to phone
                   turnAngleString = String.format(Locale.US, "Turn Angle: %.3f", turnAngle);
                   //display to phone
                   telemetry.addData("Current ", turnAngleString);
                   telemetry.update();
               }
           }
       }

   }

   //----------------------------------------------------------------------------------------------
   // Telemetry Configuration
   //----------------------------------------------------------------------------------------------

   void composeTelemetry() {

       // At the beginning of each telemetry update, grab a bunch of data
       // from the IMU that we will then display in separate lines.
       telemetry.addAction(new Runnable() {
           @Override
           public void run() {
               // Acquiring the angles is relatively expensive; we don't want
               // to do that in each of the three items that need that info, as that's
               // three times the necessary expense.
               angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
               gravity = imu.getGravity();
           }
       });

       telemetry.addLine()
               .addData("status", new Func<String>() {
                   @Override
                   public String value() {
                       return imu.getSystemStatus().toShortString();
                   }
               })
               .addData("calib", new Func<String>() {
                   @Override
                   public String value() {
                       return imu.getCalibrationStatus().toString();
                   }
               });

       telemetry.addLine()
               .addData("heading", new Func<String>() {
                   @Override
                   public String value() {
                       return formatAngle(angles.angleUnit, angles.firstAngle);
                   }
               })
               .addData("roll", new Func<String>() {
                   @Override
                   public String value() {
                       return formatAngle(angles.angleUnit, angles.secondAngle);
                   }
               })
               .addData("pitch", new Func<String>() {
                   @Override
                   public String value() {
                       return formatAngle(angles.angleUnit, angles.thirdAngle);
                   }
               });

       telemetry.addLine()
               .addData("grvty", new Func<String>() {
                   @Override
                   public String value() {
                       return gravity.toString();
                   }
               })
               .addData("mag", new Func<String>() {
                   @Override
                   public String value() {
                       return String.format(Locale.getDefault(), "%.3f",
                               Math.sqrt(gravity.xAccel * gravity.xAccel
                                       + gravity.yAccel * gravity.yAccel
                                       + gravity.zAccel * gravity.zAccel));
                   }
               });
   }

   //----------------------------------------------------------------------------------------------
   // Formatting
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
       double roll = Math.atan2( 2*(w*x +y*z), 1 - 2*(x*x + y*y)) * 180.0 / Math.PI;
       double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
       double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;

       return new double[]{yaw, pitch, roll};
   }

    public void perfectTurn(double [] initValsArray, boolean b) throws InterruptedException {
        while (b == true) {
            /***READ FINAL GYROSCOPE VALUES***/
            //read (double) gyro values after turn to do calculations with
            double[] finalValsArray = readFinalGyro();

            /***FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES***/
            double turnAngle = turnAngle(finalValsArray, initValsArray);

            //fix the turn to match X degrees
            fixTurn(turnAngle, finalValsArray, initValsArray, b);
        }
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
    
   public void fixTurn(double turnAngle, double[] finalValsArray, double[] initValsArray, boolean b) throws InterruptedException {
       //WILL PROBABLY HAVE TO ADD A LARGER ROOM FOR ERROR HERE (85-95 deg) 
       if (turnAngle > 95 || turnAngle < 85) {
               if (turnAngle < 85) {

                   TurnLeft(.5, 50);  //move wheels to compensate for turn that does not equal 90 deg

                   /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU***/
                   finalValsArray = getAngles();
                   //calculate difference from initial value AGAIN
                   turnAngle = turnAngle(finalValsArray, initValsArray);
               }
               if (turnAngle > 95) {
                   TurnRight(.5, 50); //move wheels to compensate for turn that does not equal 90 deg

                   /***CHECK IF COMPENSATION MAKES TURN EQUAL 90 DEG by reading IMU***/
                   finalValsArray = getAngles();
                   //calculate difference from initial value AGAIN
                   turnAngle = turnAngle(finalValsArray, initValsArray);
               }
           }
       else {
           b = false;
       }
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

    public void DriveForwardDistance(double power, int distance) throws InterruptedException{

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        M_drive_BL.setPower(power);
        M_drive_BR.setPower(power);
        M_drive_FL.setPower(power);
        M_drive_FR.setPower(power);

        while(opModeIsActive() && M_drive_BL.getCurrentPosition() < distance){

        }

        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        M_drive_FL.setPower(0);
        M_drive_FR.setPower(0);

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
    }

    public void DriveBackwardsDistance(double power, int distance) throws InterruptedException {
        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_drive_BL.setPower(-power);
        M_drive_BR.setPower(-power);
        M_drive_FL.setPower(-power);
        M_drive_FR.setPower(-power);

        while (opModeIsActive() && M_drive_BL.getCurrentPosition() < distance) {

        }

        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        M_drive_FL.setPower(0);
        M_drive_FR.setPower(0);

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

        M_drive_BL.setPower(power);
        M_drive_BR.setPower(-power);
        M_drive_FL.setPower(power);
        M_drive_FR.setPower(-power);

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive() && M_drive_BL.getCurrentPosition() < distance){

        }

        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        M_drive_FL.setPower(0);
        M_drive_FR.setPower(0);

        telemetry.addData("telemetry",M_drive_BL.getCurrentPosition());
        telemetry.update();

        idle();

    }

    public void TurnRight(double power, int distance)throws InterruptedException{

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_drive_BL.setPower(-power);
        M_drive_BR.setPower(power);
        M_drive_FL.setPower(-power);
        M_drive_FR.setPower(power);

        M_drive_BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while(opModeIsActive() && M_drive_BL.getCurrentPosition() < distance){

        }

        M_drive_BL.setPower(0);
        M_drive_BR.setPower(0);
        M_drive_FL.setPower(0);
        M_drive_FR.setPower(0);

        idle();
    }}



