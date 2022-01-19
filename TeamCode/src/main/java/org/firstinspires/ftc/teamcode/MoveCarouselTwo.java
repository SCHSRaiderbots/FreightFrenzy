package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveCarouselTwo {

    private DcMotorEx carouselWheel;

    private final double SPIN = -2500;
    private final double STOP = 0.0;

    public void init(HardwareMap hardwareMap) {
//Updates the status to the driver's station
        carouselWheel = hardwareMap.get(DcMotorEx.class, "motorCarousel");
        carouselWheel.setDirection((DcMotorSimple.Direction.FORWARD));
    }

    public void spin() {
        carouselWheel.setVelocity(SPIN);
    }
    public void stop_spinning() {
        carouselWheel.setVelocity(STOP);
    }

}

