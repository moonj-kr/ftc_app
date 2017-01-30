
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Jisook Moon on 1/5/17
 * State Competition
 * /

/////////Instructions///////////////

 gamepad 1:

 Drive - Joysticks UP & DOWN
 Beacon Presser - right bumper is PUSH, right trigger is RETRACT

 gamepad 2:

 Shooter - button 'b' is STOP 'a' is RUN
 Arm Release - button 'a' is RELEASE, 'b' is servo facing back up
 Lift - Controller 2: Joysticks UP & DOWN



 //Instructions
 - Fix shooter custom metal joint pieces
 - Shooter does not shoot high enough

 //Notes
 - fixed linkage issues between servos and buttons
 - arm releases are under one button
 - set initial values
 */


@TeleOp(name="Teleop_1_5", group="Linear Opmode")  // @Autonomous(...) is the other common choice


public class Teleop_1_5 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // motor declarations
    DcMotor     M_drive_BL = null,
                M_drive_BR = null,
                M_drive_FL = null,
                M_drive_FR = null,
                M_lift_FL = null,
                M_lift_FR = null,
                M_shooter = null;

    // servo declarations
    Servo       S_button_FL,
                S_liftSide_L,
                S_liftSide_R,
                S_ballDrop;

    // all of the starting servo positions
    final double ARM_INIT_POS_L = 0.8,
                 ARM_INIT_POS_R = 0.235,
                 BUTTON_INIT_POS = 0.5,
                 BALL_DROP_INIT = 0.2;

    double ARM_POS_L = ARM_INIT_POS_L,
           ARM_POS_R = ARM_INIT_POS_R,
           BUTTON_POS = BUTTON_INIT_POS,
           BALL_DROP = BALL_DROP_INIT;

    // servo constant
    double SERVO_TICK = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // mapping motor variables to their hardware counterparts
        this.M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        this.M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");
     
        this.M_drive_FL = hardwareMap.dcMotor.get("M_drive_FL");
        this.M_drive_FR = hardwareMap.dcMotor.get("M_drive_FR");

        this.M_lift_FL = hardwareMap.dcMotor.get("M_lift_FL");
        this.M_lift_FR = hardwareMap.dcMotor.get("M_lift_FR");

        this.M_shooter = hardwareMap.dcMotor.get("M_shooter");

        this.S_button_FL = hardwareMap.servo.get("S_button_FL");

        this.S_liftSide_L = hardwareMap.servo.get("S_liftSide_L");
        this.S_liftSide_R = hardwareMap.servo.get("S_liftSide_R");

        this.S_ballDrop = hardwareMap.servo.get("S_ballDrop");

        // fixing motor directions
        this.M_drive_BL.setDirection(DcMotor.Direction.FORWARD);
        this.M_drive_BR.setDirection(DcMotor.Direction.REVERSE);
        
        this.M_drive_FL.setDirection(DcMotor.Direction.FORWARD);
        this.M_drive_FR.setDirection(DcMotor.Direction.REVERSE);                                          

        this.M_lift_FL.setDirection(DcMotor.Direction.FORWARD);
        this.M_lift_FR.setDirection(DcMotor.Direction.REVERSE);

        // initializing servo positions
        this.S_liftSide_L.setPosition(ARM_INIT_POS_L);
        this.S_liftSide_R.setPosition(ARM_INIT_POS_R);
        this.S_button_FL.setPosition(BUTTON_INIT_POS);

        // wait for the game to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // motor control block
            M_drive_BL.setPower(gamepad1.right_stick_y);
            M_drive_BR.setPower(gamepad1.left_stick_y);
            M_drive_FL.setPower(gamepad1.right_stick_y);
            M_drive_FR.setPower(gamepad1.left_stick_y);
            M_lift_FL.setPower(gamepad2.right_stick_y);
            M_lift_FR.setPower(gamepad2.left_stick_y);

            // shooter control block
            if(gamepad2.b){
                M_shooter.setPower(0.0);
            }
            if(gamepad2.a){
                M_shooter.setPower(0.5);
            }
            if(gamepad2.x){
                M_shooter.setPower(-0.5);
            }

            // beacon presser control block
            if(gamepad1.right_bumper){
                BUTTON_POS -= .2;
                BUTTON_POS = Range.clip(BUTTON_POS, 0, 1);
                S_button_FL.setPosition(BUTTON_POS);
            }
            if(gamepad1.right_trigger > 0.0){
                BUTTON_POS += .2;
                S_button_FL.setPosition(BUTTON_POS);
            }

            if(gamepad1.dpad_down){
                BUTTON_POS = 0.5;
                S_button_FL.setPosition(BUTTON_POS);
            }

            // arm releases control block
            if(gamepad1.b){
                ARM_POS_L += SERVO_TICK;
                 S_liftSide_L.setPosition(ARM_POS_L);
                ARM_POS_R -= SERVO_TICK;
                S_liftSide_R.setPosition(ARM_POS_R);
            }
            if(gamepad1.a){
                ARM_POS_L -= SERVO_TICK ;
                S_liftSide_L.setPosition(ARM_POS_L);
                ARM_POS_R += SERVO_TICK;
                S_liftSide_R.setPosition(ARM_POS_R);
            }

            if(gamepad2.right_bumper){
                BALL_DROP -= SERVO_TICK;
                BALL_DROP = Range.clip(BALL_DROP, 0, 1);
                S_ballDrop.setPosition(BALL_DROP);
            }
            if(gamepad2.right_trigger > 0.0){
                BALL_DROP += SERVO_TICK;
                S_ballDrop.setPosition(BALL_DROP);
            }

            idle();
        }
    }
}


