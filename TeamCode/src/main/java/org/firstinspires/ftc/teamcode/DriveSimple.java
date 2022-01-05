package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Simple TeleOp Drive OpMode.
 * Expect this OpMode to use vanilla classes to operate various mechanisms.
 */
@TeleOp(name="DriveSimple", group="CodeDev")
public class DriveSimple extends OpMode {
    // distance sensor
    DistanceSensor distanceSensorRev2m;
    ArmMotor armMotor;

    @Override
    public void init() {
        // TODO: https://docs.ftclib.org/ftclib/

        // SERVO MAX_POSITION, MIN_POSITION

        // Want to put all the motion code in the Motion class
        Motion.init(hardwareMap);

        // Odometry
        // -- use the pose from the previous run...

        // try to get the Rev 2m distance sensor
        distanceSensorRev2m = hardwareMap.tryGet(DistanceSensor.class, "rev2meter");

        // get the end actuator
        armMotor = new ArmMotor();
        armMotor.init(hardwareMap);

        // report the initialization
        telemetry.addData("Drive motors", "initialized");
    }

    @Override
    public void init_loop() {
        // Odometry
        Motion.updateRobotPose();

        // report position
        Motion.reportPosition(telemetry);

        // if we have a 2m distance sensor
        if (distanceSensorRev2m != null) {
            // then obtain the distance
            telemetry.addData("distance", "rev 2m: %8.2f inch",
                    distanceSensorRev2m.getDistance(DistanceUnit.INCH));
        }
    }

    @Override
    public void start() {
        // Odometry
        Motion.updateRobotPose();

        // use velocity control
        Motion.setVelocity(0.0);
        Motion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the power level
        Motion.setPower(0.3);
    }

    @Override
    public void loop() {
        // Odometry
        Motion.updateRobotPose();

        // report position
        Motion.reportPosition(telemetry);

        // use the game pad to operate the intake
        if (gamepad1.a) {
            armMotor.intake();
        }
        if (gamepad1.b) {
            armMotor.outtake();
        }

        // use game pad 1 button a to reset the pose
        if (gamepad1.x) {
            Motion.setPoseInches(0.0, 0.0, 0.0);
        }

        // TODO: use abstract units
        double power = 1000.0;

        // TODO: arcade drive
        // TODO: quadratic drive

        // get the operator commands
        double powerLeft = -gamepad1.left_stick_y * power;
        double powerRight = -gamepad1.right_stick_y * power;

        Motion.setVelocity(powerLeft, powerRight);

        // report the power levels
        telemetry.addData("Drive motors", String.format((Locale)null, "%.03f %.03f", powerLeft, powerRight));

        // if we have a 2m distance sensor
        if (distanceSensorRev2m != null) {
            // then report the distance
            telemetry.addData("distance", "rev 2m: %8.2f inch",
                    distanceSensorRev2m.getDistance(DistanceUnit.INCH));
        }
    }

    @Override
    public void stop() {
    }

}
