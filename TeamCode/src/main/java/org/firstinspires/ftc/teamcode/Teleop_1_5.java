
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Jisook Moon on 1/5/17
 * /

/////////Instructions///////////////

 gamepad 1:

 Drive - Controller 1: Joysticks UP & DOWN
 Lift Servo Right Side - Controller 1: Dpad Down is FORWARD, Dpad Left is REVERSE
 Lift Servo Left Side - Controller 1: button a is FORWARD, button b is REVERSE
 Beacon Left - Controller 1: left bumper is out, Left trigger is in
 Beacon Right - Controller 1: right bumper is out, right trigger is in

 gamepad 2:

 Lift - Controller 2: Joysticks UP & DOWN
 Shooter - Controller 2: Button x is motor RUN, Button a is STOP

 //Instructions
 -

 //Note to self
 - change arm release to one button
 - fix linkage between servos and button

 */


@TeleOp(name="Teleop_1_5", group="Linear Opmode")  // @Autonomous(...) is the other common choice


public class Teleop_1_5 extends LinearOpMode {


    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor SM_drive_BL = null,
            SM_drive_BR = null,
            SM_lift_FL = null,
            SM_lift_FR = null,
            SM_shooter = null;

    Servo   SS_button_FL,
            SS_liftSide_L,
            SS_liftSide_R;

    final double ARM_INIT_POS_L = 0.8,
                 ARM_INIT_POS_R = 0.235,
                 BUTTON_INIT_POS = 0.8;

    double ARM_POS_L = ARM_INIT_POS_L,
           ARM_POS_R = ARM_INIT_POS_R,
           BUTTON_POS = BUTTON_INIT_POS;

    double SERVO_TICK = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        SM_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        SM_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");

        SM_lift_FL = hardwareMap.dcMotor.get("M_lift_FL");
        SM_lift_FR = hardwareMap.dcMotor.get("M_lift_FR");

        SM_shooter = hardwareMap.dcMotor.get("M_shooter");

        SS_button_FL = hardwareMap.servo.get("S_button_FL");

        SS_liftSide_L = hardwareMap.servo.get("S_liftSide_L");
        SS_liftSide_R = hardwareMap.servo.get("S_liftSide_R");

        SM_drive_BL.setDirection(DcMotor.Direction.FORWARD);
        SM_drive_BR.setDirection(DcMotor.Direction.REVERSE);

        SM_lift_FL.setDirection(DcMotor.Direction.FORWARD);
        SM_lift_FR.setDirection(DcMotor.Direction.REVERSE);

        SS_liftSide_L.setPosition(ARM_INIT_POS_L);
        SS_liftSide_R.setPosition(ARM_INIT_POS_R);
        SS_button_FL.setPosition(BUTTON_INIT_POS);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        telemetry.addData("left lift side", SS_liftSide_L.getPosition());
        telemetry.addData("right lift side", SS_liftSide_R.getPosition());
        telemetry.addData("button", SS_button_FL.getPosition());
        telemetry.update();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            SM_drive_BL.setPower(gamepad1.right_stick_y);
            SM_drive_BR.setPower(gamepad1.left_stick_y);

            SM_lift_FL.setPower(gamepad2.right_stick_y);
            SM_lift_FR.setPower(gamepad2.left_stick_y);

            if(gamepad2.b){
                SM_shooter.setPower(0.0);
            }

            if(gamepad2.a){
                SM_shooter.setPower(0.5);
            }

            if(gamepad1.right_bumper){
                BUTTON_POS -= SERVO_TICK;
                BUTTON_POS = Range.clip(BUTTON_POS, 0, 1); //make sure the servo's set position is not beyond 0 to 1
                SS_button_FL.setPosition(BUTTON_POS);
            }

            if(gamepad1.right_trigger > 0.0){
                BUTTON_POS += SERVO_TICK;
                SS_button_FL.setPosition(BUTTON_POS);
            }

            if(gamepad1.b){
                ARM_POS_L += SERVO_TICK;
                SS_liftSide_L.setPosition(ARM_POS_L);

                ARM_POS_R -= SERVO_TICK;
                SS_liftSide_R.setPosition(ARM_POS_R);
            }

            if(gamepad1.a){
                ARM_POS_L -= SERVO_TICK ;
                SS_liftSide_L.setPosition(ARM_POS_L);

                ARM_POS_R += SERVO_TICK;
                SS_liftSide_R.setPosition(ARM_POS_R);
            }

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}


