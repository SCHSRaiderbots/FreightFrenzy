package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MoveServo extends OpMode {
    Servo myServo;
    public final double open = 0.925;
    public final double close = 0.3;


    @Override
    public void init() {
        myServo = hardwareMap.get(Servo.class, "myServo");
        myServo.setPosition(close);
    }

    @Override
    public void loop(){
        telemetry.addData("Servo", "position "+myServo.getPosition());
        if (gamepad1.a){
            myServo.setPosition(open);
        }
        if (gamepad1.b){
            myServo.setPosition(close);
        }

    }
}
