package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Cargo Test", group="Test code")
public class CargoTest extends OpMode {

    Servo servo;

    @Override
    public void init() {
        // grab the servo
        servo = hardwareMap.get(Servo.class, "pickup");
    }

    @Override
    public void init_loop() {
        // do not need to do anything
    }

    @Override
    public void start() {
        // do not need to do anything
    }

    @Override
    public void loop() {
        // have the joystick control the servo
        double pos = gamepad1.right_trigger;
        servo.setPosition(pos);
        telemetry.addData("servo", pos);
    }

    @Override
    public void stop() {
        // do not need to do anything
    }
}
