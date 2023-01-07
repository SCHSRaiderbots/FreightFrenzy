package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    DcMotorEx motorElevator;

    /**
     * Diameter of the capstan in inches
     */
    private static final double diameterCapstan = 1.0;

    /**
     * Convert inches to ticks.
     * One rotation of the capstan is pi * D.
     * Which is also 288 ticks of a CoreHex motor.
     */
    private static final double ticksPerInch = 288.0 * (Math.PI * diameterCapstan);

    /** create an Elevator */
    public Elevator (HardwareMap hardwareMap) {
        motorElevator = hardwareMap.get(DcMotorEx.class, "elevator");

        // make sure the motor does not move
        setPower(0.0);

        setTargetPosition(5.0);
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setPower(0.3);
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

    public double getTargetPosition() {
        int ticks = motorElevator.getTargetPosition();

        return ticks / ticksPerInch;
    }

    public void stop() {
        setPower(0.0);
    }
}
