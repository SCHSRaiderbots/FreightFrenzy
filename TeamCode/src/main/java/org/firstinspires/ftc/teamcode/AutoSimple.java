package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "AutoSimple", group = "Production")
public class AutoSimple extends OpMode {

    // capture the time when Play is pressed
    double timeStart;

    @Override
    public void init() {
        // use the gamepad to set the game starting conditions
        GameConfig.init(gamepad1);
    }

    @Override
    public void init_loop() {
        // update the starting conditions
        GameConfig.init_loop(gamepad1);
        // report the starting conditions
        GameConfig.report(telemetry);
    }

    @Override
    public void start() {
        // report the start
        telemetry.addData("OpMode", "start");
        timeStart = time;
    }

    @Override
    public void loop() {
        // does nothing
        telemetry.addData("time", "%8.3f", 30.0 - (time-timeStart));
    }

    @Override
    public void stop() {
        // does nothing
    }

}
