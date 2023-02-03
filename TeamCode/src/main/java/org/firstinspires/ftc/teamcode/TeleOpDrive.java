package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * TeleOp mode for competition.
 */
@TeleOp(name="Teleop Drive", group ="Competition")
public class TeleOpDrive extends OpMode {

    // the Elevator
    Elevator elevator;
    double heightElevator = 0.0;

    Gripper gripper;

    // the Vision object
    Vision vision;

    // Whether or not to use the IMU
    boolean bIMU = false;

    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    /** the LynxModule serial number */
    String strSerialNumber;

    @Override
    public void init() {
        // get the serial number
        // TODO: use serial number to identify robot?
        strSerialNumber = LogDevice.getSerialNumber(hardwareMap);

        // report the LynxModules
        LogDevice.dumpFirmware(hardwareMap);

        // create the elevator (keeping encoder values from autonomous)
        elevator = new Elevator(hardwareMap, false);
        // create the gripper
        gripper= new Gripper(hardwareMap);

        // create the vision object
        vision = new Vision();

        // init the Vuforia localization engine
        vision.initVuforia(hardwareMap);

        // init tracking
        // TODO: does not play well with others...
        vision.initTracking();

        // we do not need an object detector...
        // build an object detector
        vision.initTfod(hardwareMap);

        // we do want to start tracking
        if (vision.targets == null) {
            Log.d("vision.targets", "is null!");
        } else {
            vision.targets.activate();
        }

        // initialize motion
        // Motion.identifyRobot(hardwareMap);
        robot = Motion.Robot.ROBOT_2022;
        Motion.init(hardwareMap);

        if (bIMU) {
            // Set up the parameters with which we will use our IMU. Note that integration
            // algorithm here just reports accelerations to the logcat log; it doesn't actually
            // provide positional information.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = false;
            parameters.loggingTag = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

            // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
            // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
            // and named "imu".
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            imu.initialize(parameters);

            composeTelemetry();
        }
    }

    @Override
    public void init_loop() {
        // report the serial number during init
        // this causes an update, so it will flash the display
        // telemetry.addData("Serial Number", strSerialNumber);

        // update the robot pose
        Motion.updateRobotPose();

        // report targets in view
        vision.reportTracking(telemetry);

    }

    @Override
    public void start() {
        // report current status

        // Motion.setPoseInches(0,0,0);

        // run using power
        Motion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        // update the robot pose
        Motion.updateRobotPose();
        Motion.reportPosition(telemetry);

        telemetry.addData("Robot", robot);

        telemetry.addData("elevator", elevator.getTargetPosition());
        telemetry.addData("current", elevator.getCurrentPosition());

        // report targets in view
        vision.reportTracking(telemetry);

        // now process the controls...

        // do some driving
        double forw = -0.7 * boost(gamepad1.left_stick_y);
        double turn = 0.4 * (gamepad1.right_stick_x);

        Motion.setPower(forw+turn, forw-turn);

        if (gamepad1.y) {
            // set the pose
            Motion.setPoseInches(Vision.inchX, Vision.inchY, Vision.degTheta);
        }

        // the elevator controls
        double p = gamepad1.left_trigger - gamepad1.right_trigger;
        // elevator.setPower(p);

        if (gamepad1.a) {
            elevator.setTargetPosition(Elevator.TargetPosition.FLOOR);
            heightElevator = elevator.getTargetPosition();
        }
        if (gamepad1.b) {
            elevator.setTargetPosition(10.0);
            heightElevator = elevator.getTargetPosition();
        }
        if (gamepad1.dpad_down) {
            elevator.setTargetPosition(Elevator.TargetPosition.GROUND);
            heightElevator = elevator.getTargetPosition();
        }
        if (gamepad1.dpad_left) {
            elevator.setTargetPosition(Elevator.TargetPosition.LOW);
            heightElevator = elevator.getTargetPosition();
        }
        if (gamepad1.dpad_right) {
            elevator.setTargetPosition(Elevator.TargetPosition.MEDIUM);
            heightElevator = elevator.getTargetPosition();
        }
        if (gamepad1.dpad_up) {
            elevator.setTargetPosition(Elevator.TargetPosition.HIGH);
            heightElevator = elevator.getTargetPosition();
        }

        // now adjust the target position
        double delta = 6.0 * (-gamepad1.left_trigger + gamepad1.right_trigger);
        elevator.setTargetPosition(heightElevator + delta);

        // left bumper closes the grip
        if (gamepad1.left_bumper){
            gripper.grip(Gripper.GripState.GRIP_CLOSED);
        }

        // right bumper opens
        if (gamepad1.right_bumper){
            gripper.grip(Gripper.GripState.GRIP_OPEN);

            // if I open the gripper, send it to the groun
            elevator.setTargetPosition(Elevator.TargetPosition.FLOOR);
            heightElevator = elevator.getTargetPosition();
        }
    }

    /**
     * Square a value retaining the sign
     * @param x
     * @return x * abs(x)
     */
    private double boost(double x) {
        return x * Math.abs(x);
    }

    @Override
    public void stop() {
        // turn off the elevator
        elevator.stop();

        // turn off tracking
        vision.targets.deactivate();
    }


    //----------------------------------------------------------------------------------------------
    // Telemetry Configuration
    //----------------------------------------------------------------------------------------------

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
