
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
            M_pickup_TOP = null,
            M_pickup_BOTTOM = null,
            M_shooter = null;

    // servo declarations
    CRServo S_button_L;
    CRServo S_button_R;
    Servo S_ballDrop_TELEOP;
    Servo S_ballDrop_AUTON;

    // all of the starting servo positions
    final double BUTTON_INIT_STOP = 0.5,
                 BALL_DROP_AUTON_INIT = 0.2,
                 BALL_DROP_TELEOP_INIT = 0.2;

    double  BUTTON_POS = BUTTON_INIT_STOP,
            BALL_DROP_AUTON_POS = BALL_DROP_AUTON_INIT,
            BALL_DROP_TELEOP_POS = BALL_DROP_TELEOP_INIT;

    // servo constant
    double SERVO_TICK = 0.03;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // mapping motor variables to their hardware counterparts
        this.M_drive_L = hardwareMap.dcMotor.get("M_drive_BL");
        this.M_drive_R = hardwareMap.dcMotor.get("M_drive_BR");
        this.M_pickup_TOP = hardwareMap.dcMotor.get("M_pickup_TOP");
        this.M_pickup_BOTTOM = hardwareMap.dcMotor.get("M_pickup_BOTTOM");
        this.M_shooter = hardwareMap.dcMotor.get("M_shooter");

        this.S_button_L = hardwareMap.crservo.get("S_button_L");
        this.S_button_R = hardwareMap.crservo.get("S_button_R");
        this.S_ballDrop_TELEOP = hardwareMap.servo.get("S_ballDrop_TELEOP");
        this.S_ballDrop_AUTON = hardwareMap.servo.get("S_ballDrop_AUTON");

        // fixing motor directions
        this.M_drive_L.setDirection(DcMotor.Direction.FORWARD);
        this.M_drive_R.setDirection(DcMotor.Direction.REVERSE);

        // initializing servo positions
        this.S_button_L.setPower(BUTTON_INIT_STOP);
        this.S_button_R.setPower(BUTTON_INIT_STOP);

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


