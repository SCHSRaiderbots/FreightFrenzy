package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp(name="DriveSimple", group="CodeDev")
public class DriveSimple extends OpMode {
    DcMotorEx motorLeft = null;
    DcMotorEx motorRight = null;

    @Override
    public void init() {
        // get the motors
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        // set the motor directions
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Odometry
        Motion.setRobotMotors(motorLeft, motorRight);
        Motion.setRobotDims2018();
        Motion.setPoseInches(0.0, 0.0, 0.0);

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

        // value for conversion
        final double power = 1.0;

        // get the operator commands
        double powerLeft = gamepad1.left_stick_y * power;
        double powerRight = gamepad1.right_stick_y * power;

        // set the motor power levels
        motorLeft.setPower(powerLeft);
        motorRight.setPower(powerRight);

        // report the power levels
        telemetry.addData("Drive motors", String.format((Locale)null, "%.03f %.03f", powerLeft, powerRight));
    }

    @Override
    public void stop() {
    }

}
