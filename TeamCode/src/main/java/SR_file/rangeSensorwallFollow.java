package SR_file;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;

@Autonomous(name = "rangeSensorwallFollow", group = "Linear Opmode")
//Disabled

/**
 * Created by Jisook on 2/16/17
 * Super Regionals
 * Autonomous for Red Alliance Starting from Corner
 */

public class rangeSensorwallFollow extends LinearOpMode {

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



    // motor powers
    final double        STOP = 0.0d;
    double              M_drivePowerR = STOP,
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

        //PID arrays
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

        int xVal, yVal, zVal = 0;     // Gyro rate Values
        int heading = 0;              // Gyro integrated heading
        int angleZ = 0;
        boolean lastResetState = false;
        boolean curResetState  = false;

        while(opModeIsActive()) {

            while (opticalDistanceSensor1.getLightDetected() < 0.09
                    && opticalDistanceSensor2.getLightDetected() < 0.09
                    && opModeIsActive()) {
                telemetry.addData("WHILE", "WHILE");
                telemetry.update();

                if (rangeSensorLeft.getDistance(DistanceUnit.CM) > 11 + 1) {
                    telemetry.addData("RANGE", "TOO BIG");
                    telemetry.update();
                    M_drive_L.setPower(0.11); //BIG BLUE .23
                    M_drive_R.setPower(0.13); //SMALL BLUE  .21

                    telemetry.addData("ANGLE IS GREATER", "ANGLE IS GREATER");
                    telemetry.addData("RANGE", rangeSensorLeft.getDistance(DistanceUnit.CM));
                    telemetry.update();
                } else if (rangeSensorLeft.getDistance(DistanceUnit.CM) < 11 - 1) {
                    telemetry.addData("RANGE", "TOO SMALL");
                    M_drive_L.setPower(0.13);
                    M_drive_R.setPower(0.11);
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

        }}}