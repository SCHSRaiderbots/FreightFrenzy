package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    DcMotorEx motorElevator;

    /**
     * Diameter of the capstan in inches
     */
    private static final double diameterCapstan = 1.4;

    /**
     * Convert inches to ticks.
     * One rotation of the capstan is pi * D.
     * Which is also 288 ticks of a CoreHex motor.
     */
    private static final double ticksPerInch = 288.0 / (Math.PI * diameterCapstan);

    /** create an Elevator */
    public Elevator (HardwareMap hardwareMap) {
        motorElevator = hardwareMap.get(DcMotorEx.class, "elevator");

        // make sure the motor does not move
        setPower(0.0);

        // TODO: only reset on Autonomous Routine
        // reset the encoder (should use a limit switch)
        setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the desired position
        setTargetPosition(0.0);

        // setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // running to position overruns at power level 1...
        // TODO: should set some reasonable PID coefficients
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(1.0);
    }

    /**
     * Simple move method
     * @param power
     */
    public void setPower(double power) {
        if (Math.abs(power) < 0.05) {
            power = 0;
        }

        motorElevator.setPower(power);
    }

    /**
     * Set the elevator run mode.
     * @param mode
     */
    public void setMode(DcMotor.RunMode mode) {
        // set the motor mode
        motorElevator.setMode(mode);
    }

    /**
     * Set the elevator's target position.
     * @param inches
     */
    public void setTargetPosition(double inches) {
        int ticks = (int)(inches * ticksPerInch);

        motorElevator.setTargetPosition(ticks);
    }

    public enum TargetPosition {
        /** position on the floor for picking up cones */
        FLOOR(0.0),
        /** ground junctions are 0.625 high */
        GROUND( 2.0),
        /** low junction is 13.5 inches */
        LOW (16.0),
        /** medium junction is 23.5 inches */
        MEDIUM(26.0),
        /** high junction is 33.5 inches */
        HIGH(36.0);

        final double height;

        /**
         * Set a named position height
         * @param h
         */
        TargetPosition(double h) {
            this.height = h;
        }
    }

    /**
     * Overloaded setTargetPosition uses enum
     * @param target
     */
    public void setTargetPosition(TargetPosition target) {
        setTargetPosition(target.height);
    }

    /**
     *
     * @return inches
     */
    public double getTargetPosition() {
        // get the motor's target position
        int ticks = motorElevator.getTargetPosition();

        return ticks / ticksPerInch;
    }

    public double getCurrentPosition() {
        return motorElevator.getCurrentPosition() / ticksPerInch;
    }

    public boolean finished() {
        return !motorElevator.isBusy();
    }

    public void stop() {
        setPower(0.0);
    }
}