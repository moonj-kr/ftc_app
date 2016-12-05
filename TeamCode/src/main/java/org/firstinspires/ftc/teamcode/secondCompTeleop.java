package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "secondCompTeleop", group = "Test")
 /*
    Updated: 12.1.16
  */


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
        M_drive_BL.setPower(gamepad1.left_stick_y);
        M_drive_BR.setPower(-gamepad1.right_stick_y);

        if(gamepad1.right_bumper) {
            S_button_FR.setPosition(S_button_FR.getPosition() + 0.01);
        }
        if(gamepad1.right_trigger > 0.0){
            S_button_FR.setPosition(S_button_FR.getPosition()-0.01);
        }

        if(gamepad1.left_bumper){
            S_button_FL.setPosition(S_button_FL.getPosition()+0.01);
        }
        if(gamepad1.left_trigger > 0.0){
            S_button_FL.setPosition(S_button_FL.getPosition()-0.01);
        }

        M_lift_FR.setPower(gamepad2.left_stick_x);
        M_lift_FL.setPower(gamepad2.left_stick_y);
    }


}
