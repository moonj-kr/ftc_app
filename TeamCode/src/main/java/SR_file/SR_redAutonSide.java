
package SR_file;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Arrays;
import java.util.Locale;

import lib.PID;

@Autonomous(name = "SR_redAutonCorner", group = "Linear Opmode")

/**
 * Created by Jisook on 2/16/17
 * Super Regionals
 * Autonomous for Red Alliance Starting from Wall: 1st Block from Corner Vortex
 */

public class SR_redAutonSide extends LinearOpMode {

    // motor declarations
    DcMotor M_drive_L = null,
            M_drive_R = null,
            M_shooter = null;

    //servo declarations
    Servo   S_button_L = null,
            S_button_R = null;
    Servo   S_ballDrop = null;

    // sensor declarations
    ColorSensor colorSensorRight; // 0x3a
    ColorSensor colorSensorLeft; // CHANGE ADDRESS
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;
    ModernRoboticsI2cGyro gyro;   // Hardware Device Object

    ModernRoboticsI2cRangeSensor rangeSensorLeft;



    final double    STOP                   = 0.0d,
            MAX_POWER              = 1.0d;
    final int       TICKS_PER_REVOLUTION   = 1120;

    final double TurnRight45 = 45.0d,
            TurnLeft45 = -45.0d;

    // motor powers
    double   M_drivePowerR = STOP,
            M_drivePowerL = STOP;
    double[] drivePowers;

    // function necessity delcarations
    int[] motorTargetsDrive;
    int[] motorTargetsTurn;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP_RIGHT = 0.5,
            BUTTON_INIT_STOP_LEFT = 0.5,
            BALL_DROP_INIT = 0.2,
            BUTTON_ADD_POS = 0.7,
            BUTTON_DEC_POS = 0.3;


    double  BUTTON_POS_R = BUTTON_INIT_STOP_RIGHT,
            BUTTON_POS_L = BUTTON_INIT_STOP_LEFT,
            BALL_DROP_POS = BALL_DROP_INIT;

    // servo constant
    double SERVO_TICK = 0.03;

    ElapsedTime clock;

    private void mapStuff() {

        // mapping motor variables to their hardware counterparts
        this.M_drive_L = hardwareMap.dcMotor.get("M_drive_L");
        this.M_drive_R = hardwareMap.dcMotor.get("M_drive_R");
        this.M_shooter = hardwareMap.dcMotor.get("M_shooter");

        this.S_button_L = hardwareMap.servo.get("S_button_L");
        this.S_button_R = hardwareMap.servo.get("S_button_R");
        this.S_ballDrop = hardwareMap.servo.get("S_ballDrop");

        // mapping sensor variables to their hardware counter parts
        colorSensorRight = hardwareMap.colorSensor.get("color_R");
        colorSensorLeft = hardwareMap.colorSensor.get("color_L");
        colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a));

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1"); //right
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");

        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

        // initializing servo positions
        this.S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
        this.S_button_R.setPosition(BUTTON_INIT_STOP_RIGHT);
        S_ballDrop.setPosition(BALL_DROP_INIT);
    }

    private void configureStuff() {
        // fixing motor directions
        this.M_drive_R.setDirection(DcMotor.Direction.REVERSE);

        // resets all the encoder values
        this.M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.M_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorTargetsDrive = new int[2];
        motorTargetsTurn = new int[2];
        Arrays.fill(motorTargetsDrive, 0);
        Arrays.fill(motorTargetsTurn, 0);

        clock = new ElapsedTime();
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

        // start calibrating the gyro.
        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        // make sure the gyro is calibrated.
        while (!isStopRequested() && gyro.isCalibrating())  {
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

        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;


        while(opModeIsActive()) {

            switch (counter) {

                /////////ACTUAL TESED AUTONOMOUS PROGRAM/////////////////////////

                case 0:

                    /*
                    //shooter run
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
                    }
                    hasBeenSet = false;

                    */
                    counter++;
                    break;

                case 1:
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    telemetry.update();

                    double deg1 = 45;
                    double i = 2;

                    while (angleZ > deg1 + i || angleZ < deg1 - i) {
                        //while(angleZ <= 22.5 ){
                        if (angleZ > deg1 + i) {
                            M_drive_L.setPower(0.05); //too slow, 18, -number to 18, 20, 18, 20
                            M_drive_R.setPower(-0.05);

                            angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                        if (angleZ < deg1 - i) {
                            M_drive_L.setPower(-0.05);
                            M_drive_R.setPower(0.05);

                            angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                    }

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    telemetry.addData("TURN COMPLETED %03d", angleZ);
                    telemetry.update();
                    counter++;
                    break;

                case 2:
                    telemetry.addData("CASE 2", "CASE 2");
                    telemetry.update();
                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    M_drive_R.setTargetPosition(5310); //295 cm
                    M_drive_L.setTargetPosition(5310);

                    M_drive_R.setPower(0.6);
                    M_drive_L.setPower(0.6);

                    M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (M_drive_L.isBusy() || M_drive_R.isBusy()) {
                    }

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                    this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                    idle();

                    counter++;
                    break;


                case 3:
                    //configureStuff(); //commented
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    telemetry.update();

                    double deg2 = -45.0;
                    double j = 2;

                    while (angleZ > deg2 + j || angleZ < deg2 - j) {
                        telemetry.addData("WHILE", "WHILE");
                        telemetry.update();
                        //while(angleZ <= 67.5 ){
                        if (angleZ > deg2 + j) {
                            M_drive_L.setPower(0.1); //too slow, 18, -number to 18, 20, 18, 20
                            M_drive_R.setPower(-0.1);

                            angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                        if (angleZ < deg2 - j) {
                            M_drive_L.setPower(-0.05);
                            M_drive_R.setPower(0.05);

                            angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }

                    }

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    telemetry.addData("TURN COMPLETED %03d", angleZ);
                    telemetry.update();
                    // gyro.resetZAxisIntegrator();

                    counter++;
                    break;


                case 4:
                    //using range sensor follow the wall
                    counter++;
                    break;

                case 5:
                    configureStuff();
                    while(opticalDistanceSensor1.getLightDetected() < 0.09 && opticalDistanceSensor2.getLightDetected() < 0.09){
                        M_drive_L.setPower(0.06);
                        M_drive_R.setPower(0.06);
                        telemetry.addData("INSIDE WHILE","INSIDE WHILE");
                        telemetry.addData("ODS 1", opticalDistanceSensor1.getLightDetected());
                        telemetry.addData("ODS 2", opticalDistanceSensor2.getLightDetected());
                        telemetry.update();
                    }
                    telemetry.addData("OUTSIDE WHILE","OUTSIDE WHILE");
                    telemetry.addData("ODS 1", opticalDistanceSensor1.getLightDetected());
                    telemetry.addData("ODS 2", opticalDistanceSensor2.getLightDetected());
                    telemetry.update();
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    counter++;
                    break;


                case 6:
                    //color sensor
                    if (colorSensorLeft.red() < colorSensorLeft.blue()) {

                        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        M_drive_R.setTargetPosition(345); //295 cm
                        M_drive_L.setTargetPosition(345);

                        M_drive_R.setPower(0.3);
                        M_drive_L.setPower(0.3);

                        M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        while (M_drive_L.isBusy() || M_drive_R.isBusy()) {
                        }

                        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                        this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                        M_drive_L.setPower(0.0);
                        M_drive_R.setPower(0.0);

                        idle();

                        //extends servo
                        S_button_L.setPosition(BUTTON_DEC_POS);
                        sleep(1300);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_L.setPosition(BUTTON_ADD_POS);
                        sleep(1300);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);

                        telemetry.addData("FIRST IF STATEMENT","FIRST");
                        telemetry.update();

                        //DRIVE TO SECOND BEACON
                    }

                    else if (colorSensorLeft.red() > colorSensorLeft.blue()) {
                        telemetry.addData("IN ELSE IF", "IN ELSE IF");
                        telemetry.addData("red", "it is red");
                        telemetry.update();

                        //extends servo
                        S_button_L.setPosition(BUTTON_DEC_POS);
                        sleep(900);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_L.setPosition(BUTTON_ADD_POS);
                        sleep(900);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
                    }
                    counter++;
                    break;

                case 7:

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    M_drive_R.setTargetPosition(3582); //295 cm
                    M_drive_L.setTargetPosition(3582);

                    M_drive_R.setPower(0.3);
                    M_drive_L.setPower(0.3);

                    M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (M_drive_L.isBusy() || M_drive_R.isBusy()) {
                    }

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                    this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    idle();

                    counter++;
                    break;

                case 8:

                    configureStuff();
                    while(opticalDistanceSensor1.getLightDetected() < 0.09 && opticalDistanceSensor2.getLightDetected() < 0.09){
                        M_drive_L.setPower(0.06);
                        M_drive_R.setPower(0.06);
                        telemetry.addData("INSIDE WHILE","INSIDE WHILE");
                        telemetry.addData("ODS 1", opticalDistanceSensor1.getLightDetected());
                        telemetry.addData("ODS 2", opticalDistanceSensor2.getLightDetected());
                        telemetry.update();
                    }
                    telemetry.addData("OUTSIDE WHILE","OUTSIDE WHILE");
                    telemetry.addData("ODS 1", opticalDistanceSensor1.getLightDetected());
                    telemetry.addData("ODS 2", opticalDistanceSensor2.getLightDetected());
                    telemetry.update();
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    counter++;
                    break;

                case 9:

                    //color sensor
                    if (colorSensorLeft.red() < colorSensorLeft.blue()) {

                        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        M_drive_R.setTargetPosition(345); //295 cm
                        M_drive_L.setTargetPosition(345);

                        M_drive_R.setPower(0.3);
                        M_drive_L.setPower(0.3);

                        M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        while (M_drive_L.isBusy() || M_drive_R.isBusy()) {
                        }

                        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                        this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                        M_drive_L.setPower(0.0);
                        M_drive_R.setPower(0.0);

                        idle();

                        //extends servo
                        S_button_L.setPosition(BUTTON_DEC_POS);
                        sleep(700);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_L.setPosition(BUTTON_ADD_POS);
                        sleep(700);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);

                        telemetry.addData("FIRST IF STATEMENT","FIRST");
                        telemetry.update();

                        //DRIVE TO SECOND BEACON
                    }

                    else if (colorSensorLeft.red() > colorSensorLeft.blue()) {
                        telemetry.addData("IN ELSE IF", "IN ELSE IF");
                        telemetry.addData("red", "it is red");
                        telemetry.update();

                        //extends servo
                        S_button_L.setPosition(BUTTON_DEC_POS);
                        sleep(900);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_L.setPosition(BUTTON_ADD_POS);
                        sleep(900);
                        S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
                    }
                    counter++;
                    break;

                case 10:

                    //configureStuff(); //commented
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    telemetry.update();

                    double deg3 = 45.0;
                    double k = 2;

                    while (angleZ > deg3 + k || angleZ < deg3 - k) {
                        telemetry.addData("WHILE", "WHILE");
                        telemetry.update();
                        //while(angleZ <= 67.5 ){
                        if (angleZ > deg3 + k) {
                            M_drive_L.setPower(0.1); //too slow, 18, -number to 18, 20, 18, 20
                            M_drive_R.setPower(-0.1);

                            angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                        if (angleZ < deg3 - k) {
                            M_drive_L.setPower(-0.05);
                            M_drive_R.setPower(0.05);

                            angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }

                    }

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    telemetry.addData("TURN COMPLETED %03d", angleZ);
                    telemetry.update();
                    // gyro.resetZAxisIntegrator();

                    counter++;
                    break;

                case 11:

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    M_drive_R.setTargetPosition(5310); //10021
                    M_drive_L.setTargetPosition(5310); //5010

                    M_drive_R.setPower(-0.6);
                    M_drive_L.setPower(-0.6);

                    M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (M_drive_L.isBusy() || M_drive_R.isBusy()) {
                    }

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


                    this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                    this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                    idle();

                    counter++;
                    break;


                default:
                    M_drivePowerR = STOP;
                    M_drivePowerL = STOP;
                    this.M_shooter.setPower(STOP);
                    telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
                    telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
                    telemetry.addData("Target R", motorTargetsDrive[0]);
                    telemetry.addData("Target L", motorTargetsDrive[1]);
                    break;
            }

            //M_drivePowerR = drivePowers[0];
            //M_drivePowerL = drivePowers[1];
            M_drive_R.setPower(M_drivePowerR);
            M_drive_L.setPower(M_drivePowerL);


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
        M_drive_R.setPower(STOP);
        M_drive_L.setPower(STOP);
    }


    public int[] setDriveTarget(double inches) {
        final double INCHES_TO_TICKS = 1100.0d / 12.0d;
        DcMotor[] motors = {M_drive_R, M_drive_L};
        int[] targets = new int[2];
        targets[0] = (int)(motors[0].getCurrentPosition() + inches * INCHES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + inches * INCHES_TO_TICKS);
        return targets;
    }

    // 12 goes 14.5

    public boolean driveForward() {
        DcMotor[] motors = {M_drive_R, M_drive_L};
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
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
            }
        }
        if(Math.abs((M_drive_R.getCurrentPosition() + M_drive_R.getCurrentPosition()) / 2.0d - motorTargetsDrive[0]) > 30) {
            return false;
        } else if(Math.abs((M_drive_L.getCurrentPosition() + M_drive_L.getCurrentPosition()) / 2.0d - motorTargetsDrive[1]) > 30) {
            return false;
        }
        return true;
    }



    private class DriveThread extends PID {
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


    public void TurnLeft(double power, int distance) throws InterruptedException{
        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_drive_R.setTargetPosition(-distance);
        M_drive_L.setTargetPosition(distance);

        M_drive_R.setPower(power);
        M_drive_L.setPower(power);

        M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_drive_L.isBusy() || M_drive_R.isBusy()) {
        }

        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();
    }

    public void TurnRight(double power, int distance)throws InterruptedException{

        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_drive_R.setTargetPosition(distance);
        M_drive_L.setTargetPosition(-distance);

        // M_drive_BL.setPower(power);
        M_drive_R.setPower(power);
        M_drive_L.setPower(power);

        M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_drive_L.isBusy() || M_drive_R.isBusy()) {

        }

        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        idle();

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

}