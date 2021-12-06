package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name="Arm", group="Linear Opmode")
public class ArmMotor extends LinearOpMode { //What does that do tho?

    private ElapsedTime runtime = new ElapsedTime();
    private Servo pickup;

    private final double S_IN = 0.0;
    private final double S_OUT = 0.5;

    //From the C_Wheel.java file for basic idea
    @Override
    public void runOpMode () {
        //tells phone fwhere the program is -^
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Connects to motor on bot and sets servo to 0
        pickup = hardwareMap.get(Servo.class, "pickup");

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            if (gamepad1.a) {
                pickup.setPosition(S_IN);
            }
            if (gamepad1.b) {
                pickup.setPosition(S_OUT);
            }
        }
    }

}
