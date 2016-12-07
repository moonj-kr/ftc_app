package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "secondCompTeleop", group = "Test")

public class secondCompTeleop extends OpMode {


    private DcMotor M_drive_BL,
                    M_drive_BR,
                    M_lift_FL,
                    M_lift_FR,
                    M_shooter;

    private Servo   S_button_FL,
                    S_button_FR,
                    S_liftSide_L,
                    S_liftSide_R;

    final double CLOSED = 0;
    final double OPEN = 1.0;

    @Override
    public void init() {
        M_drive_BL = hardwareMap.dcMotor.get("M_drive_BL");
        M_drive_BR = hardwareMap.dcMotor.get("M_drive_BR");

        M_lift_FL = hardwareMap.dcMotor.get("M_lift_FL");
        M_lift_FR = hardwareMap.dcMotor.get("M_lift_FR");

        M_shooter = hardwareMap.dcMotor.get("M_shooter");

        S_button_FL = hardwareMap.servo.get("S_button_FL");
        S_button_FR = hardwareMap.servo.get("S_button_FR");

        S_liftSide_L = hardwareMap.servo.get("S_liftSide_L");
        S_liftSide_R = hardwareMap.servo.get("S_liftSide_R");

        S_liftSide_L.setPosition(CLOSED);
        S_liftSide_R.setPosition(CLOSED);
    }

    @Override
    public void loop() {
        /*
        Drive motors
        - Controller 1
        - Joysticks
        */
        M_drive_BL.setPower(gamepad1.left_stick_x);
        M_drive_BR.setPower(-gamepad1.right_stick_y);

        /*
        Front Right Servos
        - Controller 1
        - Right Bumper extends servo
        - Right Trigger detracts servo
        */
        if(gamepad1.right_bumper) {
            S_button_FR.setPosition(S_button_FR.getPosition() + 0.01);
        }
        if(gamepad1.right_trigger > 0.0){
            S_button_FR.setPosition(S_button_FR.getPosition()-0.01);
        }

        /*
        Front Left Servos
        - Controller 1
        - Left Bumper extends servo
        - Left Trigger detracts servo
         */
        if(gamepad1.left_bumper){
            S_button_FL.setPosition(S_button_FL.getPosition()+0.01);
        }
        if(gamepad1.left_trigger > 0.0){
            S_button_FL.setPosition(S_button_FL.getPosition()-0.01);
        }

        /*
        Capball Lift Motors
        - Controller 2
        - Joysticks
         */
        M_lift_FR.setPower(gamepad2.left_stick_x);
        M_lift_FL.setPower(gamepad2.left_stick_y);

        /*
        Shooter Motor
        - Gamepad 2
        - Button b
        - Runs motor if button B is returning true value
         */
        if(gamepad2.b = true){
            M_shooter.setPower(0.5);
        }
    }

}
