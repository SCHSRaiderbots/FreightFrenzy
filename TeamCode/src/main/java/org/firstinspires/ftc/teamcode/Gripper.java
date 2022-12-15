package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Gripper {
    Servo servo;
    static final double grip_open=0;
    static final double grip_close=0.5;


    Gripper(HardwareMap hardwareMap){
        servo=hardwareMap.get(Servo.class,"ServoAJ");
    }

    public void setPosition(double position) {
        servo.setPosition(position);
    }

    public void grip(boolean state) {
        if (state)
            servo.setPosition(grip_open);
        else
            servo.setPosition(grip_close);
    }


}