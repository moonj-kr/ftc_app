
package SR_file;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * Created by Jisook Moon on 1/5/17
 * Super Regionals
 * /

 /////////Instructions///////////////

 gamepad 1:

 Drive - Joysticks UP & DOWN
 Beacon Presser - right bumper is PUSH, right trigger is RETRACT

 gamepad 2:

 Shooter - same controls


 //Notes
 - fix directions for motors
 - Also added time for servo extensions so drivers don't have to press STOP for continuous servos...
   edit times if too much or too less in sleep method
 */


@TeleOp(name="SR_teleop", group="Linear Opmode")  // @Autonomous(...) is the other common choice


public class SR_teleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // motor declarations
    DcMotor M_drive_L = null,
            M_drive_R = null,
            M_shooter = null;

    // servo declarations
    Servo S_button_L;
    Servo S_button_R;
    Servo S_ballDrop;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP_RIGHT = 0.5,
                 BUTTON_INIT_STOP_LEFT = 0.5,
                 BALL_DROP_INIT = 0.2;

    double  BUTTON_POS_R = BUTTON_INIT_STOP_RIGHT,
            BUTTON_POS_L = BUTTON_INIT_STOP_LEFT,
            BALL_DROP_POS = BALL_DROP_INIT;

    // servo constant
    double SERVO_TICK = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // mapping motor variables to their hardware counterparts
        this.M_drive_L = hardwareMap.dcMotor.get("M_drive_L");
        this.M_drive_R = hardwareMap.dcMotor.get("M_drive_R");
        this.M_shooter = hardwareMap.dcMotor.get("M_shooter");

        this.S_button_L = hardwareMap.servo.get("S_button_L");
        this.S_button_R = hardwareMap.servo.get("S_button_R");
        this.S_ballDrop = hardwareMap.servo.get("S_ballDrop");

        // fixing motor directions
        this.M_drive_L.setDirection(DcMotor.Direction.FORWARD);
        this.M_drive_R.setDirection(DcMotor.Direction.REVERSE);

        // initializing servo positions
        this.S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
        this.S_button_R.setPosition(BUTTON_INIT_STOP_RIGHT);
        S_ballDrop.setPosition(BALL_DROP_INIT);

        // wait for the game to start
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // motor control block
            M_drive_L.setPower(gamepad1.right_stick_y);
            M_drive_R.setPower(gamepad1.left_stick_y);

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
                BUTTON_POS_R +=2;
                BUTTON_POS_R = Range.clip(BUTTON_POS_R, 0, 1);
                S_button_R.setPosition(BUTTON_POS_R);
                sleep(100);
                S_button_R.setPosition(BUTTON_INIT_STOP_RIGHT);
            }
            if(gamepad1.right_trigger > 0.0){
                BUTTON_POS_R -= .2;
                S_button_R.setPosition(BUTTON_POS_R);
                sleep(100);
                S_button_R.setPosition(BUTTON_INIT_STOP_RIGHT);
            }

            // beacon presser control block
            if(gamepad1.left_bumper){
                BUTTON_POS_L -= .2;
                BUTTON_POS_L = Range.clip(BUTTON_POS_L, 0, 1);
                S_button_L.setPosition(BUTTON_POS_L);
                sleep(900);
                S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
            }
            if(gamepad1.left_trigger > 0.0){
                BUTTON_POS_L += .2;
                S_button_L.setPosition(BUTTON_POS_L);
                sleep(900);
                S_button_L.setPosition(BUTTON_INIT_STOP_LEFT);
            }

            if(gamepad2.right_bumper){
                BALL_DROP_POS -= SERVO_TICK;
                BALL_DROP_POS = Range.clip(BALL_DROP_POS, 0, 1);
                S_ballDrop.setPosition(BALL_DROP_POS);
            }
            if(gamepad2.right_trigger > 0.0){
                BALL_DROP_POS += SERVO_TICK;
                S_ballDrop.setPosition(BALL_DROP_POS);
            }

            idle();
        }
    }
}


