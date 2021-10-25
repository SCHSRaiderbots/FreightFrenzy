package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum", group="Iterative Opmode")
public class Mecanum extends OpMode {

    private DcMotorEx motorLF;
    private DcMotorEx motorLR;
    private DcMotorEx motorRF;
    private DcMotorEx motorRR;

    @Override
    public void init() {
        // find the drive motors
        motorLF = hardwareMap.get(DcMotorEx.class, "motorLF");
        motorLR = hardwareMap.get(DcMotorEx.class, "motorLR");
        motorRF = hardwareMap.get(DcMotorEx.class, "motorRF");
        motorRR = hardwareMap.get(DcMotorEx.class, "motorRR");

        motorLF.setDirection(DcMotorSimple.Direction.FORWARD);
        motorLR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRF.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void init_loop() {
        // nothing to do
    }

    @Override
    public void start() {
        // nothing to do
    }

    @Override
    public void loop() {
        double fore = -1.0 * gamepad1.right_stick_y;
        double side = -1.0 * gamepad1.right_stick_x;
        double turn = -0.3 * gamepad1.left_stick_x;

        double powerLF = fore - side - turn;
        double powerRF = fore + side + turn;
        double powerLR = fore + side - turn;
        double powerRR = fore - side + turn;

        motorLF.setPower(powerLF);
        motorRF.setPower(powerRF);
        motorLR.setPower(powerLR);
        motorRR.setPower(powerRR);
    }

    @Override
    public void stop() {

    }
}