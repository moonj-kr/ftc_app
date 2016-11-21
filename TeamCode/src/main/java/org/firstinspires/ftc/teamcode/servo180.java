package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "servo180", group = "Sensor")
@Disabled
public class servo180 extends LinearOpMode {

    Servo servo;    // Hardware Device Object
    ColorSensor colorSensor;

    double servoPosition = -1.0;



    @Override
    public void runOpMode() {

        servo = hardwareMap.servo.get("servo");
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        boolean isRed = (colorSensor.red() > colorSensor.blue());
        boolean isBlue = (colorSensor.blue() > colorSensor.red());

        // wait for the start button to be pressed.
        waitForStart();
        colorSensor.enableLed(false);

        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        //while (isRed == true) {
        while (isRed == true) {
            servo.setPosition(1.0);
            sleep(1500);
            telemetry.addData("servo", servo.getPosition());
            telemetry.update();
            isRed = false;


        }
        telemetry.addData("asdf","asdf");


    }}