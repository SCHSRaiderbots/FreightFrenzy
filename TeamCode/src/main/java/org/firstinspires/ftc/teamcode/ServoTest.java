package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import android.util.Log;

@TeleOp (name="ServoTest", group ="CodeDev")
public class ServoTest extends OpMode {
    Gripper gripper;

    @Override
    public void init(){
        gripper = new Gripper(hardwareMap);
    }

    @Override
    public void init_loop() {
        gripper.setPosition(gamepad1.left_stick_y);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        // left bumper opens the grip
        if (gamepad1.left_bumper){
            gripper.grip(true);
        }

        // right bumper closes
        if (gamepad1.right_bumper){
            gripper.grip(false);
        }
    }
}
