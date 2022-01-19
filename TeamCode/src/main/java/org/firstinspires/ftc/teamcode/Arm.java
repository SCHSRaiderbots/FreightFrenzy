package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Arm {
    private DcMotorEx armMotor;

    /**
     * Logical levels for the arm.
     */
    enum Level {
        // TODO: check values
        GROUND(0.0,0.0), LEVEL1(9.0,3.0), LEVEL2(7.5,8.5), LEVEL3(6,14.75), RETRACT(0.0,0.0);
        double radius = 5.0;
        double height=0.0;
        Level(double r,double h) {
            this.radius = r;
            this.height=h;
        }
    }

    /**
     * Correct for the extension of the arm
     * @return
     */
    double extension(Level level) {
        // TODO figure out the right values according to level
        return 3.0;
    }

    /**
     * Initialize the arm
     * @param hardwareMap device map
     */
    public void init(HardwareMap hardwareMap) {
        // get the arm motor
        armMotor = hardwareMap.get(DcMotorEx.class, "motorArm");

        // set the direction so positive numbers are up
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double rpm = 6000.0;
        double rps = rpm / 60.0;
        double ticksPerRev = 28.0;
        double f = 32000.0 / (ticksPerRev * rps);

        PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 0.1, 0.0, f, MotorControlAlgorithm.PIDF);
        PIDFCoefficients pidfR2P = new PIDFCoefficients(20.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);

        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        armMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

        // initialize the arm Motor
        int positionStart = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionStart);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        // log the motor settings
        LogDevice.dump("arm motor", armMotor);
    }

    public void zero() {
        // reset the arm encoder
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // set the position to zero
        armMotor.setTargetPosition(0);
        // set run to position mode
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // is this needed/
        armMotor.setPower(1.0);
    }

    public void setEncoder(double rel) {
        armMotor.setTargetPosition((int)(400 - rel * 400));
    }

    public void setHeightInch(double inches) {
        int pos = 37;

        armMotor.setTargetPosition(pos);
    }

    /**
     * Check if the arm has finished moving.
     * @return true if the arm has reached its position
     */
    public boolean finished() {
        // check if motor is finished
        return !armMotor.isBusy();
    }

    public void setLevel(Level level) {
       setHeightInch(level.height);
    }

}
