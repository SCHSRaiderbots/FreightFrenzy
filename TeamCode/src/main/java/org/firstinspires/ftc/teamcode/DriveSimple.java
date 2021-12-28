package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Simple TeleOp Drive OpMode.
 * Expect this OpMode to use vanilla classes to operate various mechanisms.
 */
@TeleOp(name="DriveSimple", group="CodeDev")
public class DriveSimple extends OpMode {
    DcMotorEx motorLeft = null;
    DcMotorEx motorRight = null;

    // distance sensor
    DistanceSensor distanceSensorRev2m;

    @Override
    public void init() {
        {
            // TODO: https://docs.ftclib.org/ftclib/

            // SERVO MAX_POSITION, MIN_POSITION

            LogDevice.dumpFirmware(hardwareMap);
        }

        // Want to put all the motion code in the Motion class
        Motion.init(hardwareMap);

        // get the motors
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        LogDevice.dump("motorLeft", motorLeft);
        LogDevice.dump("motorRight", motorRight);

        // TODO: may be able to fix some HD gearing issues.

        // set the motor directions
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // TODO: for UltraPlanetary
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry
        Motion.setRobotMotors(motorLeft, motorRight);
        // Motion.setRobotDims2018();
        // TODO: RobotDims
        // Motion.setRobotDims2021();
        Motion.setRobotDims2020();
        Motion.setPoseInches(0.0, 0.0, 0.0);

        // try to get the Rev 2m distance sensor
        distanceSensorRev2m = hardwareMap.tryGet(DistanceSensor.class, "rev2meter");

        // report the initialization
        telemetry.addData("Drive motors", "initialized");
    }

    @Override
    public void init_loop() {
        // Odometry
        Motion.updateRobotPose();

        // report position
        telemetry.addData("position (inches)",
                String.format((Locale)null, "(%6.01f %6.01f) %6.01f",
                        Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees));

        // if we have a 2m distance sensor
        if (distanceSensorRev2m != null) {
            // then obtain the distance
            telemetry.addData("distance", "rev 2m: %8.2f inch",
                    distanceSensorRev2m.getDistance(DistanceUnit.INCH));
        }
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

        // use game pad 1 button a to reset the pose
        if (gamepad1.a) {
            Motion.setPoseInches(0.0, 0.0, 0.0);
        }

        // value for conversion
        // double power = 200.0;

        // TODO: use abstract units
        double power = 1000.0;

        // TODO: arcade drive
        // TODO: quadratic drive

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

        // if we have a 2m distance sensor
        if (distanceSensorRev2m != null) {
            // then report the distance
            telemetry.addData("distance", "rev 2m: %8.2f inch",
                    distanceSensorRev2m.getDistance(DistanceUnit.INCH));
        }
    }

    @Override
    public void stop() {
    }

}
