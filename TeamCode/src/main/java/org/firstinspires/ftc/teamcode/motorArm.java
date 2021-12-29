package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "motorArm")
public class motorArm extends LinearOpMode {

        private ElapsedTime runtime = new ElapsedTime();
        private DcMotorEx motorArm = null;

        @Override
        public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorArm = hardwareMap.get(DcMotorEx.class, "motorArm");
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setPower(1.0);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
        }




        }
}
