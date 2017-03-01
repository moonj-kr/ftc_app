
package SR_file;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Arrays;
import java.util.Locale;


/**
 * Created by Jisook Moon on 1/5/17
 * Super Regionals
 * /

 /////////Instructions///////////////

 gamepad 1:

 Drive - Joysticks UP & DOWN
 Beacon Presser - right bumper is PUSH, right trigger is RETRACT

 gamepad 2:

 Shooter - same controls
 Ball Drop - right bumper & trigger


 //Notes
 - fix directions for motors
 - Also added time for servo extensions so drivers don't have to press STOP for continuous servos...
   edit times if too much or too less in sleep method
 - RUN RANGE SENSOR BY SWITCHING CORD OR ADDING DEVICE CORE INTERFACE
 */


@TeleOp(name="SR_sensorTeleop", group="Linear Opmode")  // @Autonomous(...) is the other common choice


public class SR_sensorTeleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // motor declarations
    DcMotor M_drive_L = null,
            M_drive_R = null,
            M_shooter = null;

    // servo declarations
    Servo S_button_L;
    Servo S_button_R;
    Servo S_ballDrop;


    // sensor declarations
    ColorSensor colorSensorRight; // 0x3a
    ColorSensor colorSensorLeft; // CHANGE ADDRESS
    OpticalDistanceSensor opticalDistanceSensor1;
    OpticalDistanceSensor opticalDistanceSensor2;

    ModernRoboticsI2cRangeSensor rangeSensorLeft;

    ModernRoboticsI2cGyro gyroSensor;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP_RIGHT = 0.5,
                 BUTTON_INIT_STOP_LEFT = 0.5,
                 BALL_DROP_INIT = 0.2;

    double  BUTTON_POS_R = BUTTON_INIT_STOP_RIGHT,
            BUTTON_POS_L = BUTTON_INIT_STOP_LEFT,
            BALL_DROP_POS = BALL_DROP_INIT;

    // servo constant
    double SERVO_TICK = 0.03;

    int xVal, yVal, zVal = 0;     // Gyro rate Values
    int heading = 0;              // Gyro integrated heading
    int angleZ = 0;
    boolean lastResetState = false;
    boolean curResetState  = false;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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
        colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a)); //change to 0x3a from 0x4c

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1"); //right
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");

        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        gyroSensor = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");

    // fixing motor directions
        this.M_drive_L.setDirection(DcMotor.Direction.FORWARD);
        this.M_drive_R.setDirection(DcMotor.Direction.REVERSE);

        // resets all the encoder values
        this.M_drive_R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_drive_L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.M_shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.M_drive_L.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.M_drive_R.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.M_shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // initializing servo positions
        this.S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
        this.S_button_R.setPosition(BUTTON_INIT_STOP_RIGHT);
        S_ballDrop.setPosition(BALL_DROP_INIT);


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

        // wait for the game to start
        waitForStart();

        while (opModeIsActive()) {

            // if the A and B buttons are pressed just now, reset Z heading.
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
            heading = gyroSensor.getHeading(); //not needed for auton
            angleZ  = gyroSensor.getIntegratedZValue();

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);
            telemetry.update();

            // motor control block
            M_drive_L.setPower(gamepad1.right_stick_y);
            M_drive_R.setPower(gamepad1.left_stick_y);

            //returns encoder values
            telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
            telemetry.addData("LF POS", M_drive_L.getCurrentPosition());

            //print joystick values
            telemetry.addData("JOYSTICK R", gamepad1.right_stick_y);
            telemetry.addData("JOYSTICK L", gamepad1.left_stick_y);
            //print distance values
            telemetry.addData("ODS 1", opticalDistanceSensor1.getLightDetected());
            telemetry.addData("ODS 2", opticalDistanceSensor2.getLightDetected());
            //print color values
            if(colorSensorLeft.red() > colorSensorLeft.blue()){
                telemetry.addData("COLOR LEFT", "IT IS RED");
            }
            if(colorSensorRight.red() > colorSensorRight.blue()){
                telemetry.addData("COLOR RIGHT", "IT IS RED");
            }
            //print range value
            telemetry.addData("range", rangeSensorLeft.getDistance(DistanceUnit.CM));

            idle();
        }
    }
}


