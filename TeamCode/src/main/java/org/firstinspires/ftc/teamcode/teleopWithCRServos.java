package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
    Telop test with continuous servos

 */
@TeleOp(name="firstCompTeleop", group="Teleop")
@Disabled

public class teleopWithCRServos extends LinearOpMode {

    /* Declare OpMode members. */

    private DcMotor M_driveBR, //back right drive motor
                    M_driveBL; //back left drive motor

    private Servo   S_beaconFR, //front right beacon servo
                    S_beaconFL; //front left beacon servo

    private static final double S_beacon_STOP = 0.5;
    private static final double S_beacon_BACKWARDS = 0.8;
    private static final double S_beacon_FORWARD = 0.2;
    private double S_beacon_POWER_RIGHT;
    private double S_beacon_POWER_LEFT;

    @Override
    public void runOpMode() throws InterruptedException
    {

        M_driveBL = hardwareMap.dcMotor.get("M_driveBL");
        M_driveBR = hardwareMap.dcMotor.get("M_driveBR");

        //mapping servo variables to their hardware counterparts
        S_beaconFL = hardwareMap.servo.get("S_beaconL");
        S_beaconFR = hardwareMap.servo.get("S_beaconR");

        //fixing motor directions
        M_driveBL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            M_driveBL.setPower(-gamepad1.left_stick_y);
            M_driveBR.setPower(-gamepad1.right_stick_y);

            if (gamepad1.right_bumper) {
                S_beacon_POWER_RIGHT = S_beacon_FORWARD;
            } else {
                S_beacon_POWER_RIGHT = S_beacon_STOP;
            }

            if (gamepad1.right_trigger > 0.0f) {
                S_beacon_POWER_RIGHT = S_beacon_BACKWARDS;
            } else {
                S_beacon_POWER_RIGHT = S_beacon_STOP;
            }

            if (gamepad1.left_bumper) {
                S_beacon_POWER_LEFT = S_beacon_FORWARD;
            } else {
                S_beacon_POWER_LEFT = S_beacon_STOP;
            }
            if (gamepad1.left_trigger > 0.0f) {
                S_beacon_POWER_LEFT = S_beacon_BACKWARDS;
            } else {
                S_beacon_POWER_LEFT = S_beacon_STOP;
            }
        }

        this.S_beaconFR.setPosition(this.S_beacon_POWER_RIGHT);
        this.S_beaconFL.setPosition(this.S_beacon_POWER_LEFT);

        telemetry.update();
        idle();
        }
    }

