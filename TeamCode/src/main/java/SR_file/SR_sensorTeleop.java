
package SR_file;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.hardware.adafruit.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


 //Notes
 - fix directions for motors
 - Also added time for servo extensions so drivers don't have to press STOP for continuous servos...
   edit times if too much or too less in sleep method
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
    ModernRoboticsI2cRangeSensor rangeSensorRight;

    // The IMU sensor object
    BNO055IMU imu;
    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP_RIGHT = 0.5,
                 BUTTON_INIT_STOP_LEFT = 0.5,
                 BALL_DROP_INIT = 0.2;

    double  BUTTON_POS_R = BUTTON_INIT_STOP_RIGHT,
            BUTTON_POS_L = BUTTON_INIT_STOP_LEFT,
            BALL_DROP_POS = BALL_DROP_INIT;

    // servo constant
    double SERVO_TICK = 0.03;

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
        //colorSensorRight.setI2cAddress(I2cAddr.create7bit(0x3a));

        opticalDistanceSensor1 = hardwareMap.opticalDistanceSensor.get("ODS1"); //right
        opticalDistanceSensor2 = hardwareMap.opticalDistanceSensor.get("ODS2");

        rangeSensorLeft = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_L");
        rangeSensorRight = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range_R");
        //change address


        //IMU Mapping Hardware
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

        // wait for the game to start
        waitForStart();
        runtime.reset();

        /***READ INITIAL GYROSCOPE VALUES***/
        //read initial (double) gyro values to do calculations with
        double[] initValsArray = getAngles();
        //store gyro values as string in order to display to phone
        String initVals = telemetrize();
        //display data to phone - can take this out later
        telemetry.addData("Data:", initVals);
        telemetry.update();

        while (opModeIsActive()) {
            // motor control block
            M_drive_L.setPower(gamepad1.right_stick_y);
            M_drive_R.setPower(gamepad1.left_stick_y);

            //returns encoder values
            telemetry.addData("RF POS", M_drive_R.getCurrentPosition());
            telemetry.addData("LF POS", M_drive_L.getCurrentPosition());
            telemetry.update();

            /***READ FINAL GYROSCOPE VALUES***/
            //read (double) gyro values after turn to do calculations with
            double[] finalValsArray = getAngles();
            //store gyro values as string in order to display to phone
            String finalVals = telemetrize();
            //display to phone - can take this out later
            telemetry.addData("All YPR Data:", finalVals);

            telemetry.addData("ODS", opticalDistanceSensor1.getLightDetected());
            telemetry.addData("ODS2", opticalDistanceSensor2.getLightDetected());
            telemetry.addData("COLOR", colorSensorRight.red());
            telemetry.addData("range", rangeSensorLeft.getDistance(DistanceUnit.CM));

            /***FIND DIFFERENCE BETWEEN FINAL AND INITIAL ANGLES***/
            double turnAngle = finalValsArray[0] - initValsArray[0];
            turnAngle = Math.abs(turnAngle);
            //convert double into string in order to display to phone
            String turnAngleString = String.format(Locale.US, "Yaw Turn Angle: %.3f", turnAngle);
            //display to phone
            telemetry.addData("Yaw Turn Angle: ", turnAngleString);
            telemetry.update();





            idle();
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




}


