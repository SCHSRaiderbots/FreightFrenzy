package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Carousel Test", group = "Concept")
public class CarouselTest extends LinearOpMode {
    public DcMotorEx motorCarousel;

    @Override
    public void runOpMode() throws InterruptedException {
        motorCarousel = hardwareMap.get(DcMotorEx.class, "motorCarousel");

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("motor", motorCarousel.getVelocity());
            motorCarousel.setVelocity(2288 * gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}
