package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
    // The drive motors
    DcMotorEx motorLeft = null;
    DcMotorEx motorRight = null;

    // capture the time when Play is pressed
    double timeStart;

    enum State {
        S_INITIAL, S_TURN, S_RUN, S_DUMP, S_TURN_PARK, S_FINAL
    }
    State state = State.S_INITIAL;

    @Override
    public void init() {
        // get the motors
        motorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
        motorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

        // set the motor directions
        motorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // TODO: put in Motion
        // set Run to Position mode
        motorLeft.setTargetPosition(motorLeft.getCurrentPosition());
        motorRight.setTargetPosition(motorRight.getCurrentPosition());
        motorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeft.setPower(0.3);
        motorRight.setPower(0.3);

        // Try to improve PIDF performance
        // TODO: pointer to documentation
        PIDFCoefficients pidfRUE = new PIDFCoefficients(10.0, 1.0, 0.0, 32000.0 / (288.0 * 125.0 / 60.0), MotorControlAlgorithm.PIDF);
        // Just use proportional here. Stalls, so add some I. Ach! Does not allow setting I.
        PIDFCoefficients pidfR2P = new PIDFCoefficients(10.0, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);

        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        motorLeft.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);
        motorRight.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

        LogDevice.dump("Left  Motor", motorLeft);
        LogDevice.dump("Right Motor", motorRight);

        // Odometry
        Motion.setRobotMotors(motorLeft, motorRight);
        Motion.setRobotDims2018();
        Motion.setPoseInches(0.0, 0.0, 0.0);

        // use the gamepad to set the game starting conditions
        GameConfig.init(gamepad1);
    }

    @Override
    public void init_loop() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // update the starting conditions
        GameConfig.init_loop(gamepad1);
        // report the starting conditions
        GameConfig.report(telemetry);

        // report the position tolerance
        telemetry.addData("Motor tolerance", "%6.02f inches", Motion.getMotorToleranceInches());
    }

    @Override
    public void start() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // report the start
        telemetry.addData("OpMode", "start");
        timeStart = time;

        // Set the starting position (x, y, theta)
        Motion.setPoseInches(GameConfig.locationStart.x,
                GameConfig.locationStart.y * GameConfig.alliance.yScale,
                GameConfig.alliance.yScale * 90.0);

        // TODO: put in Motion...
        // motorLeft.setTargetPosition(motorLeft.getCurrentPosition() + 288 * 4);
        // motorRight.setTargetPosition(motorRight.getCurrentPosition() + 288 * 4);
        // On standard field, I went 112.5 cm
        // expected 4 * pi * 9.0 cm = 113

        state = State.S_INITIAL;
        Motion.moveInches(5.0);

        // Motion.turn(90);
    }

    @Override
    public void loop() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // report the position
        telemetry.addData("time", "%8.3f, State: %s", 30.0 - (time-timeStart), state);
        telemetry.addData("pos (in)", "x: %8.1f, y: %8.1f, theta %8.1f",
                Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);
        telemetry.addData("pos (m) ", "x: %8.3f, y: %8.3f, theta %8.1f",
                Motion.xPose, Motion.yPose, Motion.thetaPoseDegrees);

        switch (state) {
            case S_INITIAL:
                // Robot is moving forward 5 inches.
                if (Motion.finished()) {
                    // turn toward the Alliance Hub
                    Motion.headTowardInches(-36.0, GameConfig.alliance.yScale * -10.0);
                    state = State.S_TURN;
                }
                break;

            case S_TURN:
                // Robot is turning toward the Alliance Hub
                if (Motion.finished()) {
                    // move toward the Alliance Hub
                    Motion.moveInches(10.0);
                    state = State.S_RUN;
                }
                break;

            case S_RUN:
                // Moving toward the Alliance Hub
                if (Motion.finished()) {
                    // back away from the Alliance Hub
                    Motion.moveInches(-10);
                    state = State.S_DUMP;
                }
                break;

            case S_DUMP:
                // backing away from the Alliance Hub
                if (Motion.finished()) {
                    // turn toward the ending location
                    Motion.headTowardInches(GameConfig.locationEnd.x, GameConfig.locationEnd.y * GameConfig.alliance.yScale);
                    state = State.S_TURN_PARK;
                }
                break;

            case S_TURN_PARK:
                // turning toward the ending location
                if (Motion.finished()) {
                    // run to the parking location
                    Motion.moveInches(20.0);
                    state = State.S_FINAL;
                }

            case S_FINAL:
                break;
        }
    }

    @Override
    public void stop() {
        // does nothing
    }

}
