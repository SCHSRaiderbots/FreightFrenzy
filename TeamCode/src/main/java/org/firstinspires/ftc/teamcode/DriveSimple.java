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
    Arm arm;
    Carousel carousel;

    // for 2020 robot
    SCHSShooter shooter = null;

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

        switch (Motion.robot) {
            case ROBOT_2020:
                // get the shooter
                shooter = new SCHSShooter();
                // initialize the shooter
                shooter.initialize(hardwareMap);
                break;

            case ROBOT_2021:
            default:

                // get the end actuator
                armMotor = new ArmMotor();
                armMotor.init(hardwareMap);

                // get the arm
                arm = new Arm();
                arm.init(hardwareMap);
                //arm.setLevel(Arm.Level.LEVEL3);
                arm.zero();

                // get the carousel
                carousel = new Carousel();
                carousel.init(hardwareMap);
                carousel.spin(0.0);

                break;
        }

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

        telemetry.addData("Robot", Motion.robot);
    }

    @Override
    public void start() {
        // Odometry
        Motion.updateRobotPose();

        // use velocity control
        Motion.setVelocity(0.0);
        Motion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the power level
        Motion.setPower(0.5);
    }

    @Override
    public void loop() {
        // Odometry
        Motion.updateRobotPose();

        // report position
        Motion.reportPosition(telemetry);

        switch (Motion.robot) {
            case ROBOT_2020:
                telemetry.addData("Shooter", "test");
                if (gamepad1.right_trigger > 0.5) {
                    // on right trigger, spin up the shooter
                    shooter.startShooters();
                    telemetry.addData("Shooter", "spin up");

                    if (gamepad1.right_bumper) {
                        // if spinning up, then we can shoot
                        shooter.servoPushRing();
                    }
                    else {
                        shooter.reloadServo();
                    }
                }
                else {
                    // if we are not spinning up, reset everything
                    shooter.stopShooters();
                    shooter.reloadServo();
                }
                break;

            case ROBOT_2021:
            default:

                // use the game pad to operate the intake
                if (gamepad2.a) {
                    armMotor.intake();
                }
                if (gamepad2.b) {
                    armMotor.outtake();
                }

                // armMotor.setPosition(-gamepad1.right_stick_y);

                // TODO: does not work
                if (false) {
                    arm.setEncoder(1 - gamepad1.left_trigger);
                    telemetry.addData("Arm ", gamepad1.left_trigger);
                } else {
                    if (gamepad2.dpad_down) arm.setLevel(Arm.Level.GROUND);
                    if (gamepad2.dpad_left) arm.setLevel(Arm.Level.LEVEL1);
                    if (gamepad2.dpad_up) arm.setLevel(Arm.Level.LEVEL2);
                    if (gamepad2.dpad_right) arm.setLevel(Arm.Level.LEVEL3);

                    telemetry.addData("Arm", "Arm height %.01f inches, %d ticks", arm.getHeightInch(), arm.armMotor.getCurrentPosition());
                }

                carousel.spin(gamepad2.right_trigger);
                telemetry.addData("Carousel", carousel.getRelativeVelocity());
                telemetry.addData("c motor", carousel.carouselMotor.getVelocity());

                break;
        }

        // use game pad 1 button x to reset the pose
        if (gamepad1.x) {
            Motion.setPoseInches(0.0, 0.0, 0.0);
        }

        // TODO: use abstract units
        // TODO: drive by power or drive by velocity choice
        double power = 2000.0;
        double velLeft;
        double velRight;
        double turn;
        double vel;

        // TODO: arcade drive
        // TODO: quadratic drive
        switch (Motion.driveMode) {
            case TANK:
                velLeft = -gamepad1.left_stick_y * power;
                velRight = -gamepad1.right_stick_y * power;
                break;

            case ARCADE_ONE_STICK:
                turn = 0.5 * gamepad1.left_stick_x * power;
                vel = -gamepad1.left_stick_y * power;
                velLeft = vel + turn;
                velRight = vel - turn;
                break;

            case ARCADE:
            default:
                turn = gamepad1.left_stick_x * power;
                vel = -gamepad1.right_stick_y * power;
                velLeft = vel + turn;
                velRight = vel - turn;
                break;
        }
        Motion.setVelocity(velLeft, velRight);

        // report the power levels
        telemetry.addData("Drive motors", String.format((Locale)null, "%.03f %.03f", velLeft, velRight));

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
