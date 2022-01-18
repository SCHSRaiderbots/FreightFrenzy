package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


public class ArmMotor{ //What does that do tho?

    private ElapsedTime runtime = new ElapsedTime();

    private Servo pickup;

    private final double S_IN = 0.1;
    private final double S_OUT = 1.0;

    //From the C_Wheel.java file for basic idea
    public void init(HardwareMap hardwareMap){
        //Connects to motor on bot and sets servo to 0
        pickup = hardwareMap.get(Servo.class, "pickup");



    }
    public void intake() {
        pickup.setPosition(S_IN);
        runtime.reset();
    }
    public void outtake() {
        pickup.setPosition(S_OUT);
        runtime.reset();

    }

    public void setPosition(double p) {
        pickup.setPosition(p);
        runtime.reset();
    }

    public boolean finished() {
        if (runtime.seconds() > 1){
            return true;
        }
        return false;
    }
}


