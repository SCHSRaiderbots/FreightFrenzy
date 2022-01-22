package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Arm {
    DcMotorEx armMotor;

    /** radius of the arm */
    final double R = 16.0;
    /** x position of the arm's axle */
    final double X1 = 1.0;
    /** y position (really z position) of the arm's axle */
    final double Y1 = 8.0;

    final int ticksZero = -200;

    /** Core Hex motor has 288 ticks per revolution */
    final double ticksPerMotorRev = 288.0;
    /** number of teeth on the driver gear */
    final double smallGear = 15.0;
    /** number of teeth on thw driven gear */
    final double bigGear = 125.0;
    /** gear down ratio */
    final double ratio = bigGear/smallGear;
    GameConfig gameConfig;


    /**
     * Logical levels for the arm.
     */
    enum Level {
        // TODO: check values
        GROUND(0.0,0.0),
        LEVEL1(9.0,3.0),
        LEVEL2(7.5,8.5),
        LEVEL3(6,14.75),
        RETRACT(0.0,16.0);
        double radius;
        double height;
        Level(double r,double h) {
            this.radius = r;
            this.height=h;
        }
    }

    /**
     * Correct for the extension of the arm
     * @return
     */
    /*void extension(Level level) {
        double armHeight;
        // TODO figure out the right values according to level
        if (gameConfig.barCode== GameConfig.BarCode.LEFT){
            setHeightInch(3.0);
        }else if(gameConfig.barCode== GameConfig.BarCode.RIGHT){
            setHeightInch(14.75);
        }
        else {
            setHeightInch(8.5);
        }

    }*/

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
        armMotor.setTargetPosition((int)(1200 - rel * 1200));
    }

    /**
     * Set the angle of the arm
     * @param theta angle in radians
     */
    public void setTheta(double theta) {
        /** number of revolutions the small gear must make to set angle theta */
        double revsSmallGear = (theta/(2*Math.PI))*(ratio);
        double ticks = ticksPerMotorRev * revsSmallGear;

        Log.d("arm ticks", "ticks are " + ticks);
        armMotor.setTargetPosition(ticksZero + (int)ticks);
    }

    public double getTheta() {
        // get the ticks relative to horizontal
        int ticks = armMotor.getCurrentPosition() - ticksZero;
        // get revs of motor
        double revsSmallGear = ticks / ticksPerMotorRev;
        // convert to revs of large gear
        double revsLargeGear = revsSmallGear / ratio;

        return revsLargeGear / (2.0 * Math.PI);
    }

    public void setHeightInch(double h) {
        double theta = Math.asin((h-X1)/R);

        Log.d("arm theta", "angle is "+ theta);
        setTheta(theta);

    }
    public double getHeightInch() {
        double theta = getTheta();

        double h = Y1 + R * Math.sin(theta);

        return h;
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
        Log.d("arm", "level is "+ level.toString());

        if (level == Level.RETRACT) {
            setTheta(135.0 * (Math.PI / 180.0));
        }
        else {
            setHeightInch(level.height);
        }
    }

}
