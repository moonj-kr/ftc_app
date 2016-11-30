
/**
 Updated: November 17, 2016
 First Competition Autonomous Red Side
 */
/*Notes:
int ANDYMARK_TICKS_PER_REV = 1120;
distance TICKS*35.1070765836
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    Updated: 11.29.16
 */

@Autonomous(name = "DriveForwardAuton", group = "Linear Opmode")

public class DriveForwardAuton extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor M_drive_BL = null;
    DcMotor M_drive_BR = null;


    int TICKS_PER_REV = 1120; //one motor rotation
    int HALF_BLOCK = 861; // about 6 inches
    int ONE_BLOCK = 3444; // about 12 inches


    @Override
    public void runOpMode() throws InterruptedException {

        M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");


        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        M_drive_BL.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        DriveFowardDistance(.5, 5426); //end time:

    }

    public void DriveFowardDistance(double power, int distance) {
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

        idle();
        //stop and change modes back to normal
        //StopDriving(power, distance);
        M_drive_BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M_drive_BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    }