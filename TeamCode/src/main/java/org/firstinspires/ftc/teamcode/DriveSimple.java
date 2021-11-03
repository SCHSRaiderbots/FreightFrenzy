package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="simple", group="simple group")
public class DriveSimple extends LinearOpMode {
    DcMotorEx motorLeft;
    DcMotorEx motorRight;

    @Override
    public void runOpMode() {
        motorLeft = hardwareMap.get(DcMotorEx.class, "motorLeft");
        motorRight = hardwareMap.get(DcMotorEx.class, "motorRight");
        // value for conversion
        final double power = 1.0;

        motorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            motorLeft.setPower(gamepad1.left_stick_y * power);
            motorRight.setPower(gamepad1.right_stick_y * power);
        }
    }

}
