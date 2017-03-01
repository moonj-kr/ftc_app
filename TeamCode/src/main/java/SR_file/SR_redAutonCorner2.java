package SR_file;


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
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Arrays;
import java.util.Locale;

@Autonomous(name = "SR_redAutonCorner2", group = "Linear Opmode")

/**
 * Created by Jisook on 2/16/17
 * Super Regionals
 * Autonomous for Red Alliance Starting from Corner
 */

public class SR_redAutonCorner2 extends LinearOpMode {

    // motor declarations
    DcMotor M_drive_L = null,
            M_drive_R = null,
            M_shooter = null;

    //servo declarations
    Servo S_button_L = null,
            S_button_R = null;
    Servo   S_ballDrop = null;

    // sensor declarations
    ColorSensor colorSensorRight; // 0x3a
    ColorSensor colorSensorLeft; // CHANGE ADDRESS
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;

    ModernRoboticsI2cRangeSensor rangeSensorLeft;
    ModernRoboticsI2cGyro gyro;   // Hardware Device Object

    //gyro constants
    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    // all of the important constants
    final double    BUTTON_INIT_POS = 0.05d;

    double BUTTON_POS = BUTTON_INIT_POS;

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
            BALL_DROP_INIT = 0.2;

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

        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

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

        this.S_button_L.setPosition(BUTTON_INIT_POS);
        this.S_button_R.setPosition(BUTTON_INIT_POS);
        this.S_ballDrop.setPosition(0.02);

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


        // if the A and B buttons are pressed just now, reset Z heading.
        //curResetState = (gamepad1.a && gamepad1.b);
        //if(curResetState && !lastResetState)  {
            gyro.resetZAxisIntegrator();
        //}
        //lastResetState = curResetState;


        waitForStart();
        clock.startTime();
        int tempMotorPosR = 0;
        int deltaMotorPos = 0;
        double increment = 0.05d;
        while(opModeIsActive()) {
            colorSensorRight.enableLed(false);

            switch (counter) {

                /////////ACTUAL TESED AUTONOMOUS PROGRAM/////////////////////////

                case 0:
                    //shooter run
                    if (!hasBeenSet) {
                        /*
                        shooterRUN(0.5, -2200); // previous -2160
                        shooterRUN(0.0, 0);
                        S_ballDrop.setPosition(1.0);
                        sleep(700); //previous 1500
                        S_ballDrop.setPosition(0.0);
                        sleep(700); //previous 1500
                        shooterRUN(0.5, -2200); //previous -2160
                        shooterRUN(0.0, 0);
                        */
                        hasBeenSet = true;
                        clock.reset();
                    }
                    hasBeenSet = false;
                    counter++;
                    break;

                case 1:
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                    // get the x, y, and z values (rate of change of angle).
                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();

                    // get the heading info.
                    // the Modern Robotics' gyro sensor keeps
                    // track of the current heading for the Z axis only.
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    telemetry.update();

                    while(angleZ <= 22.5 ){
                        M_drive_L.setPower(0.5);
                        M_drive_R.setPower(0.5);
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    while(angleZ >= 22.5){
                        M_drive_L.setPower(-0.5);
                        M_drive_R.setPower(-0.5);
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    counter++;
                    break;

                case 2:
                    //drives towards wall from turn
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(-300.0d); //-176
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 3:
                    gyro.resetZAxisIntegrator(); //may need to comment this out
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                    // get the x, y, and z values (rate of change of angle).
                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();

                    // get the heading info.
                    // the Modern Robotics' gyro sensor keeps
                    // track of the current heading for the Z axis only.
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    telemetry.update();

                    while(angleZ <= 112.5 ){
                        M_drive_L.setPower(0.5);
                        M_drive_R.setPower(0.5);
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    while(angleZ >= 112.5){
                        M_drive_L.setPower(-0.5);
                        M_drive_R.setPower(-0.5);
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    counter++;
                    break;

                case 7:
                    //drive forwards first beacon
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(60.0d); //could be positive or negative
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
                        sleep(100);
                    }
                    break;

                case 8:
                    //until optical distance sensor detects white line
                    
                    while(opticalDistanceSensor1.getLightDetected() < .40 || opticalDistanceSensor2.getLightDetected() < .40) {
                        telemetry.addData("distance", opticalDistanceSensor1.getLightDetected());
                        telemetry.update();
                        sleep(100);
                        M_drive_R.setPower(0.5d);
                        M_drive_L.setPower(0.5d);
                    }
                    M_drive_R.setPower(0.0d);
                    M_drive_L.setPower(0.0d);
                    
                    counter++;
                    break;
              
                case 9:
                    //color sensor 
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
                            telemetry.addData("R POS", M_drive_R.getCurrentPosition());
                            telemetry.addData("L POS", M_drive_L.getCurrentPosition());
                            sleep(100);
                        }
                        
                        //extends servo
                        BUTTON_POS -= .2;
                        BUTTON_POS = Range.clip(BUTTON_POS, 0, 1);
                        S_button_R.setPosition(BUTTON_POS);
                        sleep(1300);
                        S_button_R.setPosition(BUTTON_INIT_POS);
                        
                        telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
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

                            //extends servos
                            BUTTON_POS -= .2;
                            BUTTON_POS = Range.clip(BUTTON_POS, 0, 1);
                            S_button_R.setPosition(BUTTON_POS);
                            sleep(1300);
                            S_button_R.setPosition(BUTTON_INIT_POS);
                        
                            telemetry.addData("R POS", M_drive_R.getCurrentPosition());
                            telemetry.addData("L POS", M_drive_L.getCurrentPosition());
                            sleep(100);
                            counter++;
                        }

                    break;

                case 10:
                    gyro.resetZAxisIntegrator(); //may have to take this out
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                    // get the x, y, and z values (rate of change of angle).
                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();

                    // get the heading info.
                    // the Modern Robotics' gyro sensor keeps
                    // track of the current heading for the Z axis only.
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    telemetry.update();

                    while(angleZ <= 45.0 ){
                        M_drive_L.setPower(0.5);
                        M_drive_R.setPower(0.5);
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    while(angleZ >= 45.0){
                        M_drive_L.setPower(-0.5);
                        M_drive_R.setPower(-0.5);
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    counter++;
                    break;
                    
                case 11: 
                    //drives towards center vortex
                    if (!hasBeenSet) {
                        motorTargetsDrive = setDriveTarget(90.0d);
                        hasBeenSet = true;
                        clock.reset();
                    }
                    finished = driveForward();
                    if (finished || isPastTime(1.0d)) {
                        hasBeenSet = false;
                        counter++;
                        stopDriving();
                        telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
                        telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
                        sleep(100);
                    }
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
               /*motors[i].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));
               motors[i + 2].setPower(Range.clip(actualPIDValue[i], -1.0d, 1.0d));*/
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
               /*motors[i].setPower(0.0d);
               motors[i + 2].setPower(0.0d);*/
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
                //drivePowers[i] = 0.0d;
            }
        }
        if(Math.abs((M_drive_R.getCurrentPosition() + M_drive_R.getCurrentPosition()) / 2.0d - motorTargetsDrive[0]) > 30) {
            return false;
        } else if(Math.abs((M_drive_L.getCurrentPosition() + M_drive_L.getCurrentPosition()) / 2.0d - motorTargetsDrive[1]) > 30) {
            return false;
        }
        return true;
    }

    public int[] setTurnTarget(double degrees) {
        final double DEGREES_TO_TICKS = 1160.0d / 90.0d;
        DcMotor[] motors = {M_drive_R, M_drive_L};
        int[] targets = new int[2];
        //targets[0] = (int)((motors[0].getCurrentPosition() + motors[2].getCurrentPosition()) / 2 - degrees * DEGREES_TO_TICKS);
        //targets[1] = (int)((motors[1].getCurrentPosition() + motors[3].getCurrentPosition()) / 2 + degrees * DEGREES_TO_TICKS);
        targets[0] = (int)(motors[0].getCurrentPosition() - degrees * DEGREES_TO_TICKS);
        targets[1] = (int)(motors[1].getCurrentPosition() + degrees * DEGREES_TO_TICKS);
        return targets;
    }

    public boolean turnRight() {
        DcMotor[] motors = {M_drive_R, M_drive_L};
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
                if(i == 0) {
                    M_drivePowerR = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                } else {
                    M_drivePowerL = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
                }
                //drivePowers[i] = Range.clip(actualPIDValue[i], -1.0d, 1.0d);
            } else {
               /*motors[i].setPower(0.0d);
               motors[i + 2].setPower(0.0d);*/
                if(i == 0) {
                    M_drivePowerR = STOP;
                } else {
                    M_drivePowerL = STOP;
                }
                //drivePowers[i] = 0.0d;
            }
        }
        if(Math.abs((M_drive_R.getCurrentPosition() + M_drive_R.getCurrentPosition()) / 2.0d - motorTargetsTurn[0]) > 30) {
            return false;
        } else if(Math.abs((M_drive_L.getCurrentPosition() + M_drive_L.getCurrentPosition()) / 2.0d - motorTargetsTurn[1]) > 30) {
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

    /***************IMU METHODS***************/

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


}
