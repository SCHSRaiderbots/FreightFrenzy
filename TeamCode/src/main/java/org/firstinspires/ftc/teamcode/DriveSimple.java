package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp(name="DriveSimple", group="CodeDev")
public class DriveSimple extends OpMode {
    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    @Override
    public void init() {
        // get the motors
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        // set the motor directions
        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Drive motors", "initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

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
