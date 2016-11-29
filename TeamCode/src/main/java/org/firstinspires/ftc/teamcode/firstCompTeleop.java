package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by Harry and Steven on 11/14/2016.
 */
@TeleOp(name = "firstCompTeleop", group = "Test")

public class firstCompTeleop extends OpMode {


    DcMotor left;
    DcMotor right;
    Servo lefty;
    Servo righty;


    final double OPEN = 1.0;
    final double CLOSED = 0;
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("M_drive_BL");
        right = hardwareMap.dcMotor.get("M_drive_BR");


        lefty = hardwareMap.servo.get("S_button_FL");
        righty = hardwareMap.servo.get("S_button_FR");
        lefty.setPosition(CLOSED);
        righty.setPosition(CLOSED);
    }


    @Override
    public void loop() {
        right.setPower(gamepad1.left_stick_y);
        left.setPower(- gamepad1.right_stick_y);
        if(gamepad1.x && !gamepad2.y && lefty.getPosition() + 0.01 < 1.0){
            lefty.setPosition(lefty.getPosition()+0.01);
        }
        if(gamepad1.y && !gamepad2.x && lefty.getPosition() - 0.01 > 0){
            lefty.setPosition(lefty.getPosition()-0.01);
        }
        if(gamepad1.a && !gamepad2.b && righty.getPosition() + 0.01 < 1.0){
            righty.setPosition(righty.getPosition() + 0.01);
        }
        if(gamepad1.b && !gamepad2.a && righty.getPosition() - 0.01 > 0){
            righty.setPosition(righty.getPosition() - 0.01);
        }
    }
}
