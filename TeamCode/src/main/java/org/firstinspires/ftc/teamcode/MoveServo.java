package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.BasicOpMode_Linear;

@TeleOp
public class MoveServo extends OpMode {
    Servo myServo;
    public double open = 0.4;
    public double close = 0.0;


    @Override
    public void init() {
        myServo = hardwareMap.get(Servo.class, "myServo");
        myServo.setPosition(close);
    }
    public void loop(){
        if (gamepad1.a){
            myServo.setPosition(open);
        }
        if (gamepad1.b){
            myServo.setPosition(close);
        }

    }
}
