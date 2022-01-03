package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Carousel Wheel", group="Linear Opmode")
//@Disabled
public class MoveCarousel extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx carouselWheel;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
//Updates the status to the driver's station
        carouselWheel = hardwareMap.get(DcMotorEx.class, "motorCarousel");
        carouselWheel.setDirection((DcMotorSimple.Direction.FORWARD));

        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {

            double motorPower;

            if (gamepad1.a) {
                carouselWheel.setVelocity(-2500);
            }
            if (gamepad1.b) {
                carouselWheel.setVelocity(0.0);
            }

        }
    }
}

