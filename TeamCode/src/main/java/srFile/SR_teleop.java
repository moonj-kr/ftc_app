
package srFile;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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


 //Notes
 - fix directions for motors
 -
 */


@TeleOp(name="SR_teleop", group="Linear Opmode")  // @Autonomous(...) is the other common choice


public class SR_teleop extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    // motor declarations
    DcMotor M_drive_L = null,
            M_drive_R = null,
            M_shooter = null;

    // servo declarations
    CRServo S_button_L;
    CRServo S_button_R;
    Servo S_ballDrop;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP = 0.5,
                 BALL_DROP_INIT = 0.2;

    double  BUTTON_POS = BUTTON_INIT_STOP,
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

        this.S_button_L = hardwareMap.crservo.get("S_button_L");
        this.S_button_R = hardwareMap.crservo.get("S_button_R");
        this.S_ballDrop = hardwareMap.servo.get("S_ballDrop");

        // fixing motor directions
        this.M_drive_L.setDirection(DcMotor.Direction.FORWARD);
        this.M_drive_R.setDirection(DcMotor.Direction.REVERSE);

        // initializing servo positions
        this.S_button_L.setPower(BUTTON_INIT_STOP);
        this.S_button_R.setPower(BUTTON_INIT_STOP);
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
                BUTTON_POS -= .2;
                BUTTON_POS = Range.clip(BUTTON_POS, 0, 1);
                S_button_L.setPower(BUTTON_POS);
                sleep(1300);
                S_button_L.setPower(BUTTON_INIT_STOP);
            }
            if(gamepad1.right_trigger > 0.0){
                BUTTON_POS += .2;
                S_button_L.setPower(BUTTON_POS);
                sleep(1300);
                S_button_L.setPower(BUTTON_INIT_STOP);
            }

            // beacon presser control block
            if(gamepad1.left_bumper){
                BUTTON_POS -= .2;
                BUTTON_POS = Range.clip(BUTTON_POS, 0, 1);
                S_button_R.setPower(BUTTON_POS);
                sleep(1300);
                S_button_R.setPower(BUTTON_INIT_STOP);
            }
            if(gamepad1.left_trigger > 0.0){
                BUTTON_POS += .2;
                S_button_R.setPower(BUTTON_POS);
                sleep(1300);
                S_button_R.setPower(BUTTON_INIT_STOP);
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


