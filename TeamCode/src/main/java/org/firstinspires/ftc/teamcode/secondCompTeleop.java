
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 Second Competition Teleop
 December 8, 2016

 Drive - Controller 1: Joysticks UP & DOWN
 Lift - Controller 2: Joysticks UP & DOWN
 Shooter - Controller 2: Button x is motor RUN, Button a is STOP
 Lift Servo Right Side - Controller 1: Dpad Down is FORWARD, Dpad Left is REVERSE
 Lift Servo Left Side - Controller 1: button a is FORWARD, button b is REVERSE
 Beacon Left - Controller 1: left bumper is out, Left trigger is in
 Beacon Right - Controller 1: right bumper is out, right trigger is in

 */


@TeleOp(name="secondCompTeleop", group="Linear Opmode")  // @Autonomous(...) is the other common choice
@Disabled

public class secondCompTeleop extends LinearOpMode {


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor SM_drive_BL,
            SM_drive_BR,
            SM_lift_FL,
            SM_lift_FR,
            SM_shooter;

    Servo   SS_button_FL,
            SS_button_FR,
            SS_liftSide_L,
            SS_liftSide_R;

    double SERVO_TICK = 0.02;
    double servopos = 0.5;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


       /* eg: Initialize the hardware variables. Note that the strings used here as parameters
        * to 'get' must correspond to the names assigned during the robot configuration
        * step (using the FTC Robot Controller app on the phone).
        */
        SM_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        SM_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");

        SM_lift_FL = hardwareMap.dcMotor.get("M_lift_FL");
        SM_lift_FR = hardwareMap.dcMotor.get("M_lift_FR");


        SM_shooter = hardwareMap.dcMotor.get("M_shooter");

        SS_button_FL = hardwareMap.servo.get("S_button_FL");
        SS_button_FR = hardwareMap.servo.get("S_button_FR");

        SS_liftSide_L = hardwareMap.servo.get("S_liftSide_L");
        SS_liftSide_R = hardwareMap.servo.get("S_liftSide_R");


        SM_drive_BL.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        SM_drive_BR.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        SM_lift_FL.setDirection(DcMotor.Direction.FORWARD);
        SM_lift_FR.setDirection(DcMotor.Direction.REVERSE);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();







        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            SM_drive_BL.setPower(gamepad1.right_stick_y);
            SM_drive_BR.setPower(gamepad1.left_stick_y);

            SM_lift_FL.setPower(gamepad2.right_stick_y);
            SM_lift_FR.setPower(gamepad2.left_stick_y);
/*
            if(gamepad2.a){
                SM_shooter.setPower(0.0);
            }

            if(gamepad2.x){
                SM_shooter.setPower(0.5);
            }

            if(gamepad2.b){
                SM_shooter.setPower(-0.5);
            }
            */
            if(gamepad1.right_bumper) {
                servopos += SERVO_TICK;
                SS_button_FR.setPosition(servopos);
            }
            if(gamepad1.right_trigger > 0.0){
                servopos -= SERVO_TICK;
                servopos = Range.clip(servopos, 0, 1); //make sure the servo's set position is not beyond 0 to 1
                SS_button_FR.setPosition(servopos);
            }

            if(gamepad1.left_bumper){
                servopos -= SERVO_TICK;
                servopos = Range.clip(servopos, 0, 1); //make sure the servo's set position is not beyond 0 to 1
                SS_button_FL.setPosition(servopos);
            }

            if(gamepad1.left_trigger > 0.0){
                servopos += SERVO_TICK;
                SS_button_FL.setPosition(servopos);
            }


            if(gamepad2.a){
                servopos += SERVO_TICK;
                SS_liftSide_L.setPosition(servopos);
            }


            if(gamepad2.b){
                servopos -= SERVO_TICK ;
                SS_liftSide_L.setPosition(servopos);
            }

            if(gamepad2.dpad_left){
                servopos += SERVO_TICK;
                SS_liftSide_R.setPosition(servopos);
            }

            if(gamepad2.dpad_down){
                servopos -= SERVO_TICK;
                SS_liftSide_R.setPosition(servopos);
            }


            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}


