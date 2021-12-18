package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Static routines that dump information about hardware devices to the log file.
 *
 * These routines would usually be called during initialization.
 *
 * Used to explore DcMotorEx configuration (e.g., PIDF coefficients)
 * Used to describe Servo
 */
class LogDevice {
    // String used for the log
    private static final String TAG = "LogDevice";

    static void dumpFirmware(HardwareMap hardwareMap) {
        // look at all the Lynx Modules
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            // can I look at the configuration name here?

            Log.d("Module name", String.valueOf(module.getDeviceName()));
            // both are DQ2EJR1E for Riley
            Log.d("Module serial number", String.valueOf(module.getSerialNumber()));
            Log.d("Module module serial number", String.valueOf(module.getModuleSerialNumber()));
            // 3 or 4 for Riley
            Log.d("Module address", String.valueOf(module.getModuleAddress()));

            // look at the version
            String version = module.getNullableFirmwareVersionString();

            // HW: 20, Maj: 1, Min: 8, Eng: 2
            Log.d("Module firmware", version);

            // split the string into parts
            // "HW: 20, Maj: 1, Min: 8, Eng: 2"
            //  0   1   2    3  4    5  6    7
            //  firmware string -> HW 20 Maj 1 Min 8 Eng 2
            String[] parts = version.split("[ :,]+");

            // did the split work?
            if (parts != null && parts.length >= 8) {
                // turn sub-versions into integers
                int major = Integer.parseInt(parts[3]);
                int minor = Integer.parseInt(parts[5]);
                int eng = Integer.parseInt(parts[7]);

                // combine the subversion info to make a simple number
                int comb = (major << 16) | (minor << 8) | eng;

                // test against current (December 2021) version 1.8.2
                if (comb < 0x010802) {
                    Log.d("FIRMWARE", "..fails");
                }
            }
        }
    }

    static void dump(String name, HardwareDevice device) {
        Log.d(TAG, "hardware device " + name);
        Log.d(TAG, "  version: " + device.getVersion());
    }

    /**
     * Dump information about a DcMotor
     * @param name describes which motor (usually the variable name)
     * @param motor specifies the motor to describe
     */
    static void dump(String name, DcMotor motor) {
        Log.d(TAG, "motor characteristics for " + name);

        // not very interesting: just says "Motor"
        Log.d(TAG, "  device name: " + motor.getDeviceName());

        // not very interesting: just says "Lynx"
        Log.d(TAG, "  manufacturer: " + motor.getManufacturer());

        // expected this to indicate HD Hex and Core Hex, but apparently a random integer
        Log.d(TAG, "  type: " + motor.getMotorType());

        // reports into which port the device is plugged
        Log.d(TAG, "  port: " + motor.getPortNumber());

        // reports direction
        Log.d(TAG, "  direction: " + motor.getDirection());

        // reports the power
        //   I expect this value to be zero.
        //   In a PIDF mode, power is used as a limit
        Log.d(TAG, "  power: " + motor.getPower());

        // reports current position (encoder value)
        Log.d(TAG, "  position: " + motor.getCurrentPosition());
        // (dcMotor) the target position
        Log.d(TAG, "    TargetPosition: " + motor.getTargetPosition());
    }

    /**
     * Dump information about a DcMotorEx.
     * Just like DcMotor but with additional members.
     *
     * @param name describes which motor (usually the variable name)
     * @param motor specifies the motor to describe
     */
    static void dump(String name, DcMotorEx motor) {
        // first dump the ancestor
        dump(name, (DcMotor)motor);

        // Want details about the motor type...
        // These values scare me a bit: does each motor get its own instance of a motor type?
        // Or do all UltraPlanetary motors share the same motor type?
        //   here is some data from a REV Robotics Core Hex Motor
        // *** REV RoboticsUltraPlanetary HD Hex Motor
        Log.d("    motor type", motor.getMotorType().getName());
        // ticks reported as 288.0 (REV says 4 ticks/motorRev times 72)
        // *** 560 (so 20 * 28 ticks/rev)
        Log.d("    motor ticks", String.valueOf(motor.getMotorType().getTicksPerRev()));
        // max RPM 137.0 (documents say 125 RPM)
        // *** 300 (so 6000 / 20)
        Log.d("    motor rpm", String.valueOf(motor.getMotorType().getMaxRPM()));
        // achievable ticks/second 558.9599999999
        // *** 2380.
        Log.d("    motor achievable ticks/sec", String.valueOf(motor.getMotorType().getAchieveableMaxTicksPerSecond()));
        // gearing reported as 36.25!
        // *** 20
        Log.d("    gearing", String.valueOf(motor.getMotorType().getGearing()));

        // (dcMotorEx) tolerance is 5 ticks.
        // *** 5
        Log.d(TAG, "    TargetPositionTolerance = " + motor.getTargetPositionTolerance());

        // (dcMotorEx) report velocity (is this the current velocity or the target velocity?
        //   looking for analog of motor.setVelocity(v)
        //   so the interface is a bit screwy.
        //      setVelocity() -> setVelocityTarget()
        //      and there should be a getVelocityTarget()
        Log.d(TAG, "  velocity " + motor.getVelocity());

        // dump information about the PIDF coefficients
        // Velocity control
        //   4.96, 0.496, 0, 49.6
        // *** 10, 3, 0, 0, legacy
        dumpPIDF("  PIDF(rue) = ", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
        // Position control
        //   The run to position algorithm only makes use of P.
        //   See .setPIDFCoefficients()
        //   5, 0, 0, 0
        // *** 10, 0.05, 0, 0, legacy
        dumpPIDF("  PIDF(r2p) = ", motor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));
    }

    /**
     * Log information about PIDF coefficients
     * @param msg - string to identify which coefficients
     * @param pidf - the PIDF coefficients to dump
     */
    static void dumpPIDF(String msg, PIDFCoefficients pidf) {
        Log.d(TAG, msg + " " + pidf.p + ", " + pidf.i + ", " + pidf.d + ", " + pidf.f +
                " algorithm: " + pidf.algorithm);
    }

    /**
     * Dump information about a Servo
     *
     * @param name - usually the variable name for the servo
     * @param servo - the servo to describe
     */
    static void dump(String name, Servo servo) {
        Log.d(TAG, "servo information for " + name);
        // not very interesting: just says "Servo"
        Log.d(TAG, "  device name: " + servo.getDeviceName());
        // not very interesting: just says "Lynx"
        Log.d(TAG, "  manufacturer: " + servo.getManufacturer());
        // reports into which port the servo is plugged
        Log.d(TAG, "  port number: " + servo.getPortNumber());

        // direction
        Log.d(TAG, "  direction: " + servo.getDirection());

        // reports current position
        Log.d(TAG, "  position: " + servo.getPosition());
    }

    // TODO: Gamepad.Type
}