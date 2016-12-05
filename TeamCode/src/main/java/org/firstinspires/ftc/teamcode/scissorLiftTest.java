package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Updated 11.29.16
 */

@TeleOp(name = "scissorLiftTest", group = "Test")
public class scissorLiftTest extends OpMode {


    DcMotor left;
    DcMotor right;

    final double OPEN = 1.0;
    final double CLOSED = 0;
    @Override
    public void init() {
        left = hardwareMap.dcMotor.get("M_drive_BL");
        right = hardwareMap.dcMotor.get("M_drive_BR");

}
    @Override
    public void loop() {

        if (gamepad1.right_bumper) {
            right.setPower(1.0);

        }
        else if (gamepad1.left_bumper){
            left.setPower(1.0);
        }
        else if (gamepad1.right_trigger > 0.0){
            right.setPower(-1.0);
        }
        else if (gamepad1.left_trigger > 0.0){
            left.setDirection(DcMotor.Direction.REVERSE);
            left.setPower(1.0);
        }

        else if (gamepad1.x){
            right.setPower(0.0);
            left.setPower(0.0);
}

        else if (gamepad1.y){
            right.setPower(1.0);
            left.setPower(1.0);
        }

}}