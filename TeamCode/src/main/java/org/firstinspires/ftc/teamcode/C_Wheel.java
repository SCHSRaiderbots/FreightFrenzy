package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="C_Wheel", group="Linear Opmode")
//@Disabled
public class C_Wheel extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime(); //Shows how long it ran for
    private DcMotorEx carouselWheel; //Naming the motor

    @Override
    public void runOpMode () {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //tells phone where the program is -^
        carouselWheel = hardwareMap.get(DcMotorEx.class, "motorCarousel");
        carouselWheel.setDirection(DcMotorEx.Direction.FORWARD);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            double motorPower; // do the following while the button is pressed

            if (gamepad1.a) {
                carouselWheel.setVelocity(-2500);
            }

            if (gamepad1.b) {
                carouselWheel.setVelocity(0.0);
            }

            telemetry.addData("Status", "Motor Power: " + carouselWheel.getVelocity());

            telemetry.update();
        }
    }
}
