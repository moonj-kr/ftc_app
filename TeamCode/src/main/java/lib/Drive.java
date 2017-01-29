package lib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Revised 1/28/17
 */
public class Drive {
    DcMotor M_drive_FR, // front right drive motor
            M_drive_FL, // front left drive motor
            M_drive_BR, // back right drive motor
            M_drive_BL; // back left drive motor

    private class DriveThread implements Runnable {
        private final float kP = 0.25f,
                            kI = 0.25f,
                            kD = 0.25f;
        private float target;
        boolean isMoving = true,
                isFineTune = false;
        private double  timer = 0.0f,
                        fineTuneTimer = 0.0f;
        private float power = 0.0f;
        private double currDt = 0.0f;
        private float PIDValue = 0.0f;
        private float currError = 0.0f,
                        prevError = 0.0f,
                        errorRate = 0.0f,
                        accumError = 0.0f;
        private float distanceTravelled;
        private float maxPower, minPower, minPIDPower;
        private float acceptableError;
        ElapsedTime clock;



        // the main loop function
        public void run() {
            clock.startTime();
            try {
                while(isMoving) {
                    currDt = clock.time() / 1000.0d;
                    clock.reset();
                    prevError = currError;
                    currError = target - distanceTravelled;
                    errorRate = prevError - currError;
                    accumError += errorRate * currDt;
                    PIDValue = kP * currError + kI * accumError;
                    PIDValue = Range.clip(PIDValue, -maxPower, maxPower);
                    power = -PIDValue;
                    if(currError < acceptableError) {
                        power = 0.0f;
                        isMoving = false;
                    }
                    Thread.sleep(10);
                    // add in fine tune mode later
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                return;
            }
        }

    }
}