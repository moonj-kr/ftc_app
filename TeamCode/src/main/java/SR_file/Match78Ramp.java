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

import java.util.Arrays;

@Autonomous(name = "Match78Ramp", group = "Linear Opmode")

/**
 * Created by Jisook on 2/16/17
 * Super Regionals
 * Autonomous for Red Alliance Starting from Corner
 * Launches two particles & parks on ramp
 * Using time for turns
 */

public class Match78Ramp extends LinearOpMode {

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
        clock.startTime();

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

                    M_drive_R.setPower(0.5);
                    M_drive_L.setPower(0.5);

                    M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    while (M_drive_L.isBusy() || M_drive_R.isBusy() && opModeIsActive()) {
                    }

                    M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added
                    this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //added

                    counter++;
                    break;

                case 1:
                    // change motor to rotate little more
                    // first turn from initial position

                    counter++;
                    break;
                    //launches particle

                case 2:
                    shooterRUN(0.6, -2200); // previous -2160
                    shooterRUN(0.0, 0);
                    S_ballDrop.setPosition(1.0);
                    sleep(950); //previous 1500
                    S_ballDrop.setPosition(0.0);
                    sleep(950); //previous 1500
                    shooterRUN(0.6, -2200); //previous -2160
                    shooterRUN(0.0, 0);
                    counter++;
                    break;

                case 3:

                    M_drive_R.setPower(-0.3);
                    M_drive_L.setPower(0.3);

                    sleep(500);

                    M_drive_L.setPower(0.0);
                    M_drive_R.setPower(0.0);
                    counter++;
                    break;

                case 4:
                    
                   telemetry.addData("CASE 0", "CASE 0");
                   telemetry.update();
                   M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                   M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                   M_drive_R.setTargetPosition(7000); //295 cm
                   M_drive_L.setTargetPosition(7000); //10021 - 740 = 9281 735 + 635 1

                   M_drive_R.setPower(0.5);
                   M_drive_L.setPower(0.5);

                   M_drive_L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   M_drive_R.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                   while ((M_drive_L.isBusy() || M_drive_R.isBusy()) && opModeIsActive()) {
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
                    break;
            }

        }}

    public void shooterRUN(double power, int distance) {
        M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        M_shooter.setTargetPosition(distance);

        M_shooter.setPower(power);

        M_shooter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (M_shooter.isBusy() && opModeIsActive()) {
            //wait
        }
    }
}
