package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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
    DcMotorEx motorLeft = null;
    DcMotorEx motorRight = null;

    // capture the time when Play is pressed
    double timeStart;

    /** actuator at the end of the arm */
    ArmMotor armMotor;

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

        // Odometry
        Motion.setRobotMotors(motorLeft, motorRight);
        Motion.setRobotDims2018();
        Motion.setPoseInches(0.0, 0.0, 0.0);

        // use the gamepad to set the game starting conditions
        GameConfig.init();

        // get the actuator at the end of the arm
        armMotor = new ArmMotor();
        armMotor.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // update the starting conditions
        GameConfig.init_loop(gamepad1);
        // report the starting conditions
        GameConfig.report(telemetry);
    }

    @Override
    public void start() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // report the start
        telemetry.addData("OpMode", "start");
        timeStart = time;

        // TODO: put in Motion...
        // Motion.moveInches(5.0);

        Motion.turnDegrees(90);

        armMotor.intake();
    }

    @Override
    public void loop() {
        // keep updating the robot pose
        Motion.updateRobotPose();

        // does nothing
        telemetry.addData("time", "%8.3f", 30.0 - (time-timeStart));
        telemetry.addData("pos (in)", "x: %8.1f, y: %8.1f, theta %8.1f",
                Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);
        telemetry.addData("pos (m) ", "x: %8.3f, y: %8.3f, theta %8.1f",
                Motion.xPose, Motion.yPose, Motion.thetaPoseDegrees);
    }

    @Override
    public void stop() {
        // does nothing
    }

}
