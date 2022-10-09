package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

/* copied this class from previous year; was named SCHSShooterNew */
class SCHSShooter {

    private final double SHOOT_VEL = 950.0;

    // the devices
    private DcMotorEx shooterMotorLeft;
    private DcMotorEx shooterMotorRight;
    private Servo shooterServo;

    private boolean isShooting = false;

    public void initialize(HardwareMap hardwareMap) {
        // get th devices
        shooterMotorLeft = hardwareMap.get(DcMotorEx.class, "shooterMotor1");
        shooterMotorRight = hardwareMap.get(DcMotorEx.class, "shooterMotor2");
        shooterServo = hardwareMap.get(Servo.class, "shooterServo");

        shooterMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);
        shooterServo.setDirection(Servo.Direction.FORWARD);

        // assume max velocity of 90 rps gives 2^15 PWM
        // changing this does not have much effect
        double Fshoot = 32767.0 / (90.0 * 28.0);

        // 10, 3 gives 6 seconds with ringing
        // 50, 5 gives 3 seconds with overshoot
        // 20, 2 gives 4 seconds with overshoot
        // 50, 2 gives 6 seconds
        // 50, 15 gives 4 seconds
        PIDFCoefficients pidfRUEshoot = new PIDFCoefficients(50.0, 5, 0, Fshoot, MotorControlAlgorithm.PIDF);
        // shooter does not use R2P...
        // PIDFCoefficients pidfR2P = new PIDFCoefficients(10, 0.05, 0, 0, MotorControlAlgorithm.PIDF);

        // set the PIDF coefficients
        shooterMotorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUEshoot);
        shooterMotorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUEshoot);

        // make sure the shooter is shut down
        shooterMotorLeft.setVelocity(0.0);
        shooterMotorRight.setVelocity(0.0);

        // we will use velocity control
        shooterMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // shooterMotorLeft.setPower(1.0);
        // shooterMotorRight.setPower(1.0);

        // describe the motor...
        // Default PIDF:
        //   PIDF(rue) 10, 3, 0, 0, Legacy
        //   PIDF(r2p) 10, 0.05, 0, 0, Legacy
        LogDevice.dump("shooterMotorLeft", shooterMotorLeft);
        LogDevice.dump("shooterMotorRight", shooterMotorRight);
    }

    // TODO: this is not the way to control the shooter...
    // shoot 3 rings at start of auto
    public void shoot(double velocity) {
        isShooting = true;
        startShooters();
        sleep(3500);
        servoPushRing();
        sleep(1500);
        reloadServo();
        sleep(1500);
        servoPushRing();
        sleep(1500);
        reloadServo();
        sleep(1500);
        servoPushRing();
        sleep(1500);
        reloadServo();
        stopShooters();
        isShooting = false;
    }

    //check if robot is still shooting
    public boolean getIsShooting() {
        return isShooting;
    }

    //check is shooter is not moving
    public boolean isShooterStopped() {
        if (shooterMotorLeft.getVelocity() == 0 && shooterMotorRight.getVelocity() == 0 ) {
            return true;
        }
        return false;
    }

    // start the shooter motors spinning
    public void startShooters() {
        shooterMotorLeft.setVelocity(SHOOT_VEL + 100);
        shooterMotorRight.setVelocity(SHOOT_VEL - 100 - 100);
    }

    public void stopShooters() {
        shooterMotorLeft.setVelocity(0);
        shooterMotorRight.setVelocity(0);
    }

    public void servoPushRing() {
        shooterServo.setPosition(0.2);
    }

    public void reloadServo() {
        shooterServo.setPosition(0);
    }
}