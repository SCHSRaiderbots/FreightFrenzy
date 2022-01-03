package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "motorArmTeleOp")
public class motorArmTeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorArmTeleOp = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorArmTeleOp = hardwareMap.get(DcMotorEx.class, "motorArm");
        motorArmTeleOp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArmTeleOp.setTargetPosition(0);
        motorArmTeleOp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArmTeleOp.setPower(1.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            motorArmTeleOp.setTargetPosition((int)(gamepad1.left_trigger * 100));
        }




    }
}
