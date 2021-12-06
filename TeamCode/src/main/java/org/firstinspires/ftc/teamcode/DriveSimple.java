package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp(name="DriveSimple", group="CodeDev")
public class DriveSimple extends OpMode {
    DcMotorEx motorLeft = null;
    DcMotorEx motorRight = null;

    // assume the firmware is OK
    boolean boolFirmware = true;

    @Override
    public void init() {
        {
            // TODO: https://docs.ftclib.org/ftclib/

            // TODO: getUserConfigureName();
            // where is it located?

            // SERVO MAX_POSITION, MIN_POSITION

            // look at all the Lynx Modules
            // USBAccessibleLynxModule ???
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

                if (version == null) {
                    // nothing to do but cry
                    boolFirmware = false;
                } else {
                    // split the string into parts
                    // "HW: 20, Maj: 1, Min: 8, Eng: 2"
                    //  0   1   2    3  4    5  6    7
                    //  firmware string -> HW 20 Maj 1 Min 8 Eng 2
                    String[] parts = version.split("[ :,]+");

                    // turn subversions into integers
                    int major = Integer.parseInt(parts[3]);
                    int minor = Integer.parseInt(parts[5]);
                    int eng = Integer.parseInt(parts[7]);

                    // combine the subversion info to make a simple number
                    int comb = (major << 16) | (minor << 8) | eng;

                    // test against 1.8.2
                    if (comb < 0x010802) {
                        Log.d("FIRMWARE", "..fails");
                        boolFirmware = false;
                    }
                }
            }
        }


        // get the motors
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        LogDevice.dump("motorLeft", motorLeft);
        LogDevice.dump("motorRight", motorRight);

        // TODO: may be able to fix some HD gearing issues.

        // set the motor directions
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // Odometry
        Motion.setRobotMotors(motorLeft, motorRight);
        Motion.setRobotDims2018();
        Motion.setPoseInches(0.0, 0.0, 0.0);

        // report the initialization
        telemetry.addData("Drive motors", "initialized");
    }

    @Override
    public void init_loop() {
        // Odometry
        Motion.updateRobotPose();

        // report position
        telemetry.addData("position",
                String.format((Locale)null, "%6.01f %6.01f %6.01f",
                        Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees));
    }

    @Override
    public void start() {
        // Odometry
        Motion.updateRobotPose();
    }

    @Override
    public void loop() {
        // Odometry
        Motion.updateRobotPose();

        // report position
        telemetry.addData("position",
                String.format((Locale)null, "%6.01f %6.01f %6.01f",
                        Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees));

        // use gamepad1.a to reset the pose
        if (gamepad1.a) {
            Motion.setPoseInches(0.0, 0.0, 0.0);
        }

        // value for conversion
        final double power = 200.0;

        // get the operator commands
        double powerLeft = -gamepad1.left_stick_y * power;
        double powerRight = -gamepad1.right_stick_y * power;

        // set the motor power levels
        motorLeft.setPower(0.3);
        motorRight.setPower(0.3);

        motorLeft.setVelocity(powerLeft);
        motorRight.setVelocity(powerRight);

        // report the power levels
        telemetry.addData("Drive motors", String.format((Locale)null, "%.03f %.03f", powerLeft, powerRight));
    }

    @Override
    public void stop() {
    }

}
