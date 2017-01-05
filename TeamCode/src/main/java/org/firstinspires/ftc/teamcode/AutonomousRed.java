
/**

 Second Competition Autonomous Red Side:
 - Drives using encoder methods
 - MR Optical Distance for white line
 - MR Color Sensors for beacon light detection
 -MR Range Sensor for distance from wall after turn

Additional Notes: ANDYMARK_TICKS_PER_REV = 1120;
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

    ColorSensor colorSensorLeft; //0x3c
    ColorSensor colorSensorRight; //different address 0x3a

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;

    ModernRoboticsI2cRangeSensor rangeSensorLeft;


    boolean LEDState = false;

    int TICKS_PER_REV = 1120; //one motor rotation
    int HALF_BLOCK = 861; // about 6 inches
    int ONE_BLOCK = 3444; // about 12 inches

    @Override
    public void runOpMode() throws InterruptedException {

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");

        S_button_FL = hardwareMap.servo.get("S_button_FL");
        S_button_FR = hardwareMap.servo.get("S_button_FR");

        colorSensorLeft = hardwareMap.colorSensor.get("color_FL");
        colorSensorRight = hardwareMap.colorSensor.get("color_FR");

        colorSensorLeft.setI2cAddress(I2cAddr.create7bit(0x3c)); //check if 7 bit is a problem if an error occurs
        colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a));

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
/*
Today's goals:
Finish testing gyro
-Configure robot

- Test servo vaules 1 to -1
- Test distance of robot per rotation
- Test angles of robot per distance
- Decide if gyro is a necessity

 */
/*
        //servo testing
        S_button_FL.setPosition(0.0);
        sleep(1000); //OUT LEFT

        S_button_FL.setPosition(1.0);
        sleep(1000); //IN LEFT

        S_button_FR.setPosition(1.0);
        sleep(1000); //OUT RIGHT

        S_button_FR.setPosition(0.0);
        sleep(1000); //IN RIGHT
*/
        DriveFowardDistance(.5,3604);

        TurnRight(.5,937);                        //end time:
        //turn 45 degrees //TURNS LEFT
        //1875/2 = 937

        TurnLeft(.5, 937);



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
