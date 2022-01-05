package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class CaroWheel{
    private DcMotorEx turn;
    public final double MAX_VELOCIY = -2500.0;

    public void init(HardwareMap hardwareMap){
        turn = hardwareMap.get(DcMotorEx.class, "motorCarousel");
        turn.setDirection(DcMotorEx.Direction.FORWARD);
    }
    public void turnWheel(double velocity){
        turn.setVelocity(velocity);
    }
}
