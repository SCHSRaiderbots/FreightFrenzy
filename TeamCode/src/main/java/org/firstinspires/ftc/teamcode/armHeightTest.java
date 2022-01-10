package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "armHeightTest", group = "CodeDev")
public class armHeightTest extends LinearOpMode {
    public DcMotorEx armMotor;
    int level = 1;
    int positionStart;

    @Override
    public void runOpMode() {
        armMotor = hardwareMap.get(DcMotorEx.class, "motorArm");
        positionStart = armMotor.getCurrentPosition();
        armMotor.setTargetPosition(positionStart);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1.0);

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a) {
                armMotor.setTargetPosition(positionStart + 20);

            }

            else if (gamepad1.b){
                armMotor.setTargetPosition(positionStart + 30);
            }

            else if (gamepad1.x){
                armMotor.setTargetPosition(positionStart + 40);
            }


        }
    }
}

