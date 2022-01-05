package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * A simple Autonomous routine.
 * Goals during Autonomous:
 *   configure starting conditions
 *   detect the Duck (or Team Shipping Element / TSE)
 *   move to alliance shipping hub
 *   deposit preloaded freight (6 points) at appropriate level (10 or 20 points)
 *   get clear of alliance shipping hub
 *   move to Warehouse (10) or Storage Unit (6)
 */
@Autonomous(name = "AutoSimple", group = "Production")
public class AutoSimple extends OpMode {
    // capture the time when Play is pressed
    double timeStart;

    /** actuator at the end of the arm */
    ArmMotor armMotor;

    enum State {STATE_INITIAL, STATE_RUNNING, STATE_TURNING, STATE_FINAL}
    State state = State.STATE_INITIAL;

    @Override
    public void init() {
        // initialize Motion operations
        Motion.init(hardwareMap);

        // Odometry
        Motion.setPoseInches(0.0, 0.0, 0.0);

        // use the gamepad to set the game starting conditions
        GameConfig.init();

        // get the actuator at the end of the arm
        armMotor = new ArmMotor();
        armMotor.init(hardwareMap);
        armMotor.outtake();
    }

    @Override
    public void init_loop() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // update the starting conditions
        GameConfig.init_loop(gamepad1);
        // report the starting conditions
        GameConfig.report(telemetry);

        // report the current bar code
        telemetry.addData("Barcode", GameConfig.barCode);
    }

    @Override
    public void start() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // remember start time
        timeStart = time;

        // set initial state
        state = State.STATE_INITIAL;

        armMotor.intake();
    }

    @Override
    public void loop() {
        // keep updating the robot pose
        Motion.updateRobotPose();
        // report the position
        Motion.reportPosition(telemetry);

        // show that the variable time updates...
        telemetry.addData("time", "%8.3f", 30.0 - (time-timeStart));

        telemetry.addData("State", state);
        switch (state) {
            case STATE_INITIAL:
                // start moving forward
                Motion.moveInches(20.0);
                state = State.STATE_RUNNING;
                break;

            case STATE_RUNNING:
                // we are running forward
                if (Motion.finished()) {
                    Motion.turnDegrees(-90.0);
                    state = State.STATE_TURNING;
                }
                break;

            case STATE_TURNING:
                // we are turning right
                if (Motion.finished()) {
                    state = State.STATE_FINAL;
                }
                break;

            case STATE_FINAL:
                // we are sitting pretty
                break;
        }
    }

    @Override
    public void stop() {
        // does nothing
    }

}
