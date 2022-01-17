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
    enum Level {GROUND, LEVEL1, LEVEL2, LEVEL3, RETRACT}

    /**
     * Initialize the arm
     * @param hardwareMap device map
     */
    public void init(HardwareMap hardwareMap) {
        // get the arm motor
        armMotor = hardwareMap.get(DcMotorEx.class, "motorArm");

        // set the direction so positive numbers are up
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double f = 32000.0 / (28.0 * 6000.0 / 60.0);

        PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 1.0, 0.0, f, MotorControlAlgorithm.PIDF);
        PIDFCoefficients pidfR2P = new PIDFCoefficients(10.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);

        // initialize the arm Motor
        int positionStart = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionStart);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);
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
        switch (level) {
            case GROUND:
                // setHeightInch(0.0);
                setEncoder(1.0);
                break;
            case LEVEL1:
                // setHeightInch(3.0);
                setEncoder(.85);
                break;
            case LEVEL2:
                // setHeightInch(8.0);
                setEncoder(.77);
                break;
            case LEVEL3:
                // setHeightInch(12.0);
                setEncoder(.61);
                break;
            case RETRACT:
                // setHeightInch(12.0);
                setEncoder(0.0);
                break;
        }
    }

}
