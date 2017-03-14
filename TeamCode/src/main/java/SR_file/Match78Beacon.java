package SR_file;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name = "Match78Beacon", group = "Linear Opmode")

/**
 * Created by Jisook on 2/16/17
 * Super Regionals
 * Autonomous for Red Alliance Starting from Corner
 */

public class Match78Beacon extends LinearOpMode {

    // motor declarations
    DcMotor M_drive_L = null,
            M_drive_R = null,
            M_shooter = null;

    //servo declarations
    Servo   S_button_L = null,
            S_button_R = null;
    Servo   S_ballDrop = null;

    // sensor declarations
    //ColorSensor colorSensorRight; // address: 0x3a
    ColorSensor colorSensorLeft;
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;
    ModernRoboticsI2cGyro gyro;
    ModernRoboticsI2cRangeSensor rangeSensorLeft;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP_RIGHT = 0.5,
            BUTTON_INIT_STOP_LEFT = 0.5,
            BALL_DROP_INIT = 0.2,
            BUTTON_ADD_POS = 0.7,
            BUTTON_DEC_POS = 0.3;

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
        //colorSensorRight = hardwareMap.colorSensor.get("color_R");
        colorSensorLeft = hardwareMap.colorSensor.get("color_L");

        // I2C address change
        //colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a));

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1"); //right
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2"); //left

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

        // encoder
        this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.M_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        clock = new ElapsedTime();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        mapStuff();
        configureStuff();

        ////////////////////////// run auton stuff starts here ///////////////////////////////
        int counter = 0;
        waitForStart();

        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        while(opModeIsActive()) {

            switch (counter) {

                /////////ACTUAL TESED AUTONOMOUS PROGRAM/////////////////////////

                case 0:
                    
                    telemetry.addData("CASE 0", "CASE 0");
                    telemetry.update();
                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    M_drive_R.setTargetPosition(635); //295 cm
                    M_drive_L.setTargetPosition(635); //10021 - 740 = 9281

                    M_drive_R.setPower(0.75);
                    M_drive_L.setPower(0.75);

                    M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while ((M_drive_L.isBusy() || M_drive_R.isBusy()) && opModeIsActive()) {
                    }

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    idle();

                    this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                    this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                    telemetry.addData("CASE 0", "End of case 0");
                    telemetry.update();
                    counter++;
                    break;

                case 1:

                    telemetry.addData("CASE 1", "Start of case 1");
                    telemetry.update();
                    shooterRUN(0.6, -2200); // previous -2160
                    shooterRUN(0.0, 0);
                    //sleep(2000);
                    telemetry.addData("SHOOTER ","SHOOTER END");
                    telemetry.update();
                    counter++;
                    break;

               case 2:
// change motor to rotate little more
                   // first turn from initial position

                   counter++;
                   break;

                case 3:
                    // drive towards wall
                    sleep(3000);
                    telemetry.addData("CASE 3", "CASE 3");
                    telemetry.update();
                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    M_drive_R.setTargetPosition(9001); //295 cm
                    M_drive_L.setTargetPosition(9001); //9851 - 740 = 9111

                    M_drive_R.setPower(0.5); //original = .65
                    M_drive_L.setPower(0.5);

                    M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while ((M_drive_L.isBusy() || M_drive_R.isBusy())&& opModeIsActive()) {
                    }

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                    this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                    counter++;
                    break;
                    
                case 4:
                    // change motor to rotate little more
                    // first turn from initial position

                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);
                    finalAngle = 0;
                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    //telemetry.update();

                    deg = -18.5; //18.5
                    b= 1;

                    while ((angleZ > deg + b || angleZ < deg - b) && opModeIsActive()) {
                        //while(angleZ <= 22.5 ){
                        angleZ  = gyro.getIntegratedZValue();
                        finalAngle = angleZ;

                        if (angleZ > deg + b) {
                            M_drive_L.setPower(0.05); //too slow, 18, -number to 18, 20, 18, 20
                            M_drive_R.setPower(-0.05);

                            // angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                        if (angleZ < deg - b) {
                            M_drive_L.setPower(-0.05);
                            M_drive_R.setPower(0.05);

                            //angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                    }

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    telemetry.addData("TURN COMPLETED %03d", angleZ);
                    angleZ_two  = gyro.getIntegratedZValue();
                    telemetry.addData("After Ang. %03d", angleZ_two);
                    telemetry.addData("After While Ang. %03d", finalAngle);
                    telemetry.update();
                    counter++;
                    break;

                case 5:
                    // change motor to rotate little more
                    // first turn from initial position

                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Beg. Ang. %03d", angleZ);

                     finalAngle = 0;
                    xVal = gyro.rawX();
                    yVal = gyro.rawY();
                    zVal = gyro.rawZ();
                    heading = gyro.getHeading();
                    angleZ  = gyro.getIntegratedZValue();
                    telemetry.addData("1", "Int. Ang. %03d", angleZ);
                    //telemetry.update();

                     deg = 41.0; //18.5
                     b= 1;

                    while (angleZ > deg + b || angleZ < deg - b) {
                        //while(angleZ <= 22.5 ){
                        angleZ  = gyro.getIntegratedZValue();
                        finalAngle = angleZ;

                        if (angleZ > deg + b) {
                            M_drive_L.setPower(0.05); //too slow, 18, -number to 18, 20, 18, 20
                            M_drive_R.setPower(-0.05);

                            // angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                        if (angleZ < deg - b) {
                            M_drive_L.setPower(-0.05);
                            M_drive_R.setPower(0.05);

                            //angleZ  = gyro.getIntegratedZValue();
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("1", "Int. Ang. %03d", angleZ);
                            telemetry.update();
                        }
                    }

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    telemetry.addData("TURN COMPLETED %03d", angleZ);
                     angleZ_two  = gyro.getIntegratedZValue();
                    telemetry.addData("After Ang. %03d", angleZ_two);
                    telemetry.addData("After While Ang. %03d", finalAngle);
                    telemetry.update();
                    counter++;
                    break;

                case 6:
                    // stop when white line is detected
                    // range sensor follow the wall

                    while ((opticalDistanceSensor1.getLightDetected() < 0.09) //only checking one
                            && opModeIsActive()) {
                        telemetry.addData("WHILE", "WHILE");
                        telemetry.update();

                        if (rangeSensorLeft.getDistance(DistanceUnit.CM) > 9 + 1) {
                            telemetry.addData("RANGE", "TOO BIG");
                            telemetry.update();
                            M_drive_L.setPower(0.24); //BIG BLUE .23
                            M_drive_R.setPower(0.21); //SMALL BLUE  .21

                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("RANGE", rangeSensorLeft.getDistance(DistanceUnit.CM));
                            telemetry.update();
                        } else if (rangeSensorLeft.getDistance(DistanceUnit.CM) < 9 - 1) {
                            telemetry.addData("RANGE", "TOO SMALL");
                            M_drive_L.setPower(0.21);
                            M_drive_R.setPower(0.24);
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("RANGE", rangeSensorLeft.getDistance(DistanceUnit.CM));
                            telemetry.update();
                        }

                        else {
                            M_drive_L.setPower(0.23);
                            M_drive_R.setPower(0.23);
                        }
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);

                    counter++;
                    break;

                case 7:
                    telemetry.addData("CASE 7", "CASE 7");
                    // detects color for first beacon
                    // presses beacon if red

                    if (colorSensorLeft.red() > colorSensorLeft.blue()) {
                        telemetry.addData("RED", "RED");
                        telemetry.update();
                        M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                        M_drive_R.setTargetPosition(340); //295 cm
                        M_drive_L.setTargetPosition(340);

                        M_drive_R.setPower(0.5);
                        M_drive_L.setPower(0.5);

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

                        //extends servo
                        S_button_R.setPosition(BUTTON_ADD_POS);
                        sleep(1200);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_R.setPosition(BUTTON_DEC_POS);
                        sleep(1200);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);

                        telemetry.addData("FIRST IF STATEMENT","FIRST");
                        telemetry.update();

                        //DRIVE TO SECOND BEACON
                    }

                    else if (colorSensorLeft.red() < colorSensorLeft.blue()) {
                        telemetry.addData("IN ELSE IF", "IN ELSE IF");
                        telemetry.addData("BLUE", "it is BLUE");
                        telemetry.update();

                        //extends servo
                        S_button_R.setPosition(BUTTON_ADD_POS);
                        sleep(1200);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_R.setPosition(BUTTON_DEC_POS);
                        sleep(1200);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);
                    }
                    counter++;
                    break;

                case 8:
                    //drives to second beacon


                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    M_drive_R.setTargetPosition(302); //295 cm
                    M_drive_L.setTargetPosition(302);

                    M_drive_R.setPower(0.5);
                    M_drive_L.setPower(0.5);

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

                    counter ++;
                    break;

                case 9:
                    // stops at white line
                    // checks range to see if it is 15

                    while ((opticalDistanceSensor1.getLightDetected() < 0.09) && opModeIsActive()) {
                        telemetry.addData("WHILE", "WHILE");
                        telemetry.update();

                        if (rangeSensorLeft.getDistance(DistanceUnit.CM) > 9 + 1) {
                            telemetry.addData("RANGE", "TOO BIG");
                            telemetry.update();
                            M_drive_L.setPower(0.24); //BIG BLUE .23
                            M_drive_R.setPower(0.21); //SMALL BLUE  .21

                            telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                            telemetry.addData("RANGE", rangeSensorLeft.getDistance(DistanceUnit.CM));
                            telemetry.update();
                        } else if (rangeSensorLeft.getDistance(DistanceUnit.CM) < 9 - 1) {
                            telemetry.addData("RANGE", "TOO SMALL");
                            M_drive_L.setPower(0.21);
                            M_drive_R.setPower(0.24);
                            telemetry.addData("ANGLE IS LESSER", "ANGLE IS LESSER");
                            telemetry.addData("RANGE", rangeSensorLeft.getDistance(DistanceUnit.CM));
                            telemetry.update();
                        }

                        else {
                            M_drive_L.setPower(0.23);
                            M_drive_R.setPower(0.23);
                        }
                    }
                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    counter++;
                    break;

                case 10:
                    // detects beacon color
                    // presses beacon if red
                    telemetry.addData("CASE 7", "CASE 7");
                    telemetry.update();

                    if (colorSensorLeft.red() > colorSensorLeft.blue()) {

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
                        
                        //extends servo
                        S_button_R.setPosition(BUTTON_ADD_POS);
                        sleep(1200);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_R.setPosition(BUTTON_DEC_POS);
                        sleep(1200);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);

                        telemetry.addData("FIRST IF STATEMENT","FIRST");
                        telemetry.update();

                        //DRIVE TO SECOND BEACON
                    }

                    else if (colorSensorLeft.red() < colorSensorLeft.blue()) {
                        telemetry.addData("IN ELSE IF", "IN ELSE IF");
                        telemetry.addData("red", "it is red");
                        telemetry.update();

                        //extends servo
                        S_button_R.setPosition(BUTTON_ADD_POS);
                        sleep(1300);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);
                        telemetry.addData("RETRACTING NOW","RETRACTING");
                        telemetry.update();
                        S_button_R.setPosition(BUTTON_DEC_POS);
                        sleep(1300);
                        S_button_R.setPosition(BUTTON_INIT_STOP_LEFT);
                    }
                    counter++;
                    break;
                    
                default:
                    M_drivePowerR = STOP;
                    M_drivePowerL = STOP;
                    this.M_shooter.setPower(STOP);
                    telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
                    telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
                    break;
            }

        }}

    public void shooterRUN(double power, int distance) {
        telemetry.addData("Shooter", "Starting shot");
        telemetry.update();
        
        M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_shooter.setTargetPosition(distance);

        M_shooter.setPower(power);

        M_shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while ((M_shooter.isBusy())&& opModeIsActive) {
          
        }
        telemetry.addData("Shooter", "Finished shot");
        telemetry.update();
    }
}
