package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Locale;

/**
 * This class consolidates some of the robot motion calculations.
 * It uses static methods because there is only one physical robot.
 * Furthermore, we want to carry the robot's current position from one OpMode to the next.
 * For example, an Autonomous routine might set a particular starting position.
 * At the end of the Autonomous routine, the robot will have an updated position.
 * By using static variables, the succeeding TeleOp routine will know the robot's position.
 * <p>
 * This code is borrowing from 2019, 2020, and 2021 robot code
 * </p>
 * <p>
 * So the OpMode should:</p>
 * <ol>
 * <li>tell this class which motors are being used:
 *   Motion.setRobot(DcMotorLeft, DCMotorRight);</li>
 * <li>tell this class the robot dimensions:
 *   Motion.setRobot2019();</li>
 * <li>and then make periodic calls to update the robot pose:
 *   Motion.updateRobotPose()</li>
 * <li>the current position may be accessed with
 *   Motion.xPose, Motion.yPose, Motion.thetaPose</li>
 * </ol>
 */
public class Motion {

    // The CoreHex motor has 4 ticks per revolution and is geared down by 72
    //   those attributes should be in the DcMotor class
    public static final double CORE_HEX_TICKS_PER_REV = 4 * 72.0;

    // The HD Hex Motor has 56 ticks per revolution
    //    so claims http://www.revrobotics.com/content/docs/HDMotorEncoderGuide.pdf
    //   Apparently, we see 1/2 the counts that REV claims (28 instead of 56)
    public static final double HD_HEX_TICKS_PER_REV = (56.0 / 2);

    // The HD motor has 20:1 and 40:1 gearboxes
    //    the 20:1 is geared 20 to 1
    //    the 40:1 is geared 40 to 1

    // The HD Hex Motor is also used with the UltraPlanetary cartridges.
    // These values are used to calculate actual gear ratios
    // The ring gear has 55 teeth
    // The 3:1 cartridge is actually 84:29 (2.9...) = (55+29)/29
    public static final double HD_HEX_GEAR_CART_3_1 = 84.0/29.0;
    // The 4:1 cartridge is actually 76:21 (3.6...) = (55+21)/21
    public static final double HD_HEX_GEAR_CART_4_1 = 76.0/21.0;
    // The 5:1 cartridge is actually 68:13 (5.2...) = (55+13)/13
    public static final double HD_HEX_GEAR_CART_5_1 = 68.0/13.0;

    /**
     * Identity and Info about the actual robot
     * properties?
     *   wheel diameter
     *   gear ratio? information for ticks per wheel rotation
     *   wheel half separation
     *   width
     *   length
     *   center of rotation
     *   camera location (x, y, z, rx, ry, rz)
     */
    public enum Robot {
        /** Rover Ruckus */
        ROBOT_2018,
        /** Skystone */
        ROBOT_2019,
        /** Ultimate Goal */
        ROBOT_2020,
        /** Freight Frenzy Robot */
        ROBOT_2021,
        /** PowerPlay */
        ROBOT_2022,
        ROBOT_MECANUM
    }
    /** The robot being used. Defaults to ROBOT_2021. */
    public static Robot robot = Robot.ROBOT_2022;

    // robot parameters

    // the wheel diameters are 90mm nominal
    /** Left wheel diameter (meters) */
    static private double mWheelDiameterLeft = 0.090;
    /** Right wheel diameter (meters) */
    static private double mWheelDiameterRight = 0.090;

    /** Half the distance between the left and right wheels (meters) */
    static double distWheel = (14.0 - (1.0/16.0)) * 0.0254 / 2;

    /** motor tick per wheel revolution */
    static double ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

    // derived robot parameters
    // Distance per tick
    //   leaving the units vague at this point

    // the distance per tick for each wheel = circumference / ticks
    /** Distance traveled per encoder tick for the left wheel (meters/tick) */
    static private double distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
    /** Distance traveled per encoder tick for the right wheel (meters/tick) */
    static private double distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);

    // the robot pose
    //   can have .updatePose(), .getPose(), .setPose()
    //   using static should allow the Pose to be carried over from Autonomous to TeleOp
    //     Autonomous can set the initial pose
    //     When TeleOp starts, it can use the existing Pose
    //        If there was no autonomous, then initial Pose is random
    //        A button press during TeleOp's init_loop could set a known Pose
    /** pose x-position in meters */
    static double xPose = 0.0;
    /** pose y-position in meters */
    static double yPose = 0.0;
    /** pose angle in radians */
    static double thetaPose = 0.0;

    // Inch versions of the robot pose
    static double xPoseInches = 0.0;
    static double yPoseInches = 0.0;
    static double thetaPoseDegrees = 0.0;

    /** Width of a floor tile */
    static final double metersPerTile = 23.625 * 0.0254;


    // motors used to drive the differential robot
    static private DcMotorEx dcmotorLeft;
    static private DcMotorEx dcmotorRight;

    // encoder counts
    // There's a subtle issue here
    //    If robot is not moving, it is OK to set these values to the current encoder counts
    //    That could always happen during .init()
    static private int cEncoderLeft;
    static private int cEncoderRight;

    // choose a drive mode
    enum DriveMode {TANK, ARCADE, ARCADE_ONE_STICK}
    static DriveMode driveMode = DriveMode.ARCADE;

    /**
     * Look at the hardwareMap to figure out which robot is being used.
     * @param hardwareMap the robot configuration information
     */
    static void identifyRobot(HardwareMap hardwareMap) {

        // Use a touch sensor (that need not exist) to identify the robot
        RevTouchSensor touch;

        // if it has a robot2018 touch sensor, then it is a 2018 robot...
        touch = hardwareMap.tryGet(RevTouchSensor.class, "robot2018");
        if (touch != null) {
            // found the 2018 robot
            robot = Robot.ROBOT_2018;
            return;
        }

        touch = hardwareMap.tryGet(RevTouchSensor.class, "robot2020");
        if (touch != null) {
            // found the 2020 robot
            robot = Robot.ROBOT_2020;
        }

        // we did not find any evidence to contrary, assume robot was set correctly...
    }

    /**
     * Initialize the robot motion information.
     * @param hardwareMap the robot configuration information
     */
    static void init(HardwareMap hardwareMap) {
        // dump the firmware
        LogDevice.dumpFirmware(hardwareMap);

        // TODO: don't identify the robot within this code...
        // identify the robot
        // identifyRobot(hardwareMap);

        // initialize the robot
        switch (robot) {
            case ROBOT_MECANUM:
                // give up on mecanum
                break;

            case ROBOT_2018:
                // get the motors
                dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
                dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

                // set the motor directions
                dcmotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                dcmotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

                // set the robot dimensions
                setRobotDims2018();

                // TODO: lost the PIDF code...
                // dcmotorRight.getMotorType()
                //  .getMaxRPM(); -> 137 ,, 300 == 6000/20
                //  .getTicksPerRev(); -> 288,, 560 == 20 * 28
                //  .getAchieveableMaxTicksPerSecond 559,, 2380
                double rpm = 120.0;
                double revsPerSecond = rpm / 60.0;
                double ticksPerRev = 288.0;
                double f = 32000.0 / (ticksPerRev * revsPerSecond);
                PIDFCoefficients pidfRUE = new PIDFCoefficients(10, 1, 0, f, MotorControlAlgorithm.PIDF);
                PIDFCoefficients pidfR2P = new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF);
                setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
                setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);
                break;

            case ROBOT_2019:
                // get the motors
                dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
                dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

                // set the motor directions
                dcmotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                dcmotorRight.setDirection(DcMotorSimple.Direction.FORWARD);

                setRobotDims2019();
                break;

            case ROBOT_2020:
                // get the motors
                dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
                dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

                // set the motor directions
                dcmotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                dcmotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

                setRobotDims2020();
                break;

            case ROBOT_2021:
                // get the motors
                dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
                dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

                // set the motor directions
                dcmotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                dcmotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

                setRobotDims2021();


                rpm = 6000.0;

                revsPerSecond = rpm / 60.0;
                ticksPerRev = 56.0;
                f = 32000.0 / (ticksPerRev * revsPerSecond);
                pidfRUE = new PIDFCoefficients(10, 1, 0, f, MotorControlAlgorithm.PIDF);
                pidfR2P = new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF);
                setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
                setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

                setMotorToleranceInches(0.3);

                break;

            case ROBOT_2022:
            default:
                // get the motors
                dcmotorLeft = hardwareMap.get(DcMotorEx.class, "leftMotor");
                dcmotorRight = hardwareMap.get(DcMotorEx.class, "rightMotor");

                // set the motor directions
                dcmotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                dcmotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

                setRobotDims2022();


                rpm = 6000.0;

                revsPerSecond = rpm / 60.0;
                ticksPerRev = 56.0;
                f = 32000.0 / (ticksPerRev * revsPerSecond);
                pidfRUE = new PIDFCoefficients(10, 1, 0, f, MotorControlAlgorithm.PIDF);
                pidfR2P = new PIDFCoefficients(10, 0, 0, 0, MotorControlAlgorithm.PIDF);
                setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
                setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

                setMotorToleranceInches(0.3);

                break;
        }

        // remember the current encoder counts to do odometry
        // DcMotor Direction also affects the encoder counts
        // remember the current encoder counts
        // Should always do this (even if not resetting the Pose)
        cEncoderLeft = dcmotorLeft.getCurrentPosition();
        cEncoderRight = dcmotorRight.getCurrentPosition();

        // TODO: take this out
        // set Run to Position mode
        dcmotorLeft.setTargetPosition(dcmotorLeft.getCurrentPosition());
        dcmotorRight.setTargetPosition(dcmotorRight.getCurrentPosition());
        setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the power level
        setPower(0.3);

        // dump the motors
        LogDevice.dump("dcmotorLeft", dcmotorLeft);
        LogDevice.dump("dcmotorRight", dcmotorRight);
    }

    /**
     * Odometry must know which motors are being used...
     */
    static void setRobotMotors(DcMotorEx mLeft, DcMotorEx mRight)
    {
        // remember the motors.
        dcmotorLeft = mLeft;
        dcmotorRight = mRight;

        // remember the current encoder counts to do odometry
        // DcMotor Direction also affects the encoder counts
        // remember the current encoder counts
        // Should always do this (even if not resetting the Pose)
        cEncoderLeft = dcmotorLeft.getCurrentPosition();
        cEncoderRight = dcmotorRight.getCurrentPosition();
    }

    /**
     * Set the drive motor power levels.
     * @param powerLeft power level for left motor (-1 to 1)
     * @param powerRight power level for right motor (-1 to 1)
     */
    static void setPower(double powerLeft, double powerRight) {
        // set the power level for both motors
        dcmotorLeft.setPower(powerLeft);
        dcmotorRight.setPower(powerRight);
    }

    /**
     * Set the drive motor power levels.
     * @param power power level (-1 to 1)
     */
    static void setPower(double power) {
        // set the power level
        setPower(power, power);
    }

    /**
     * Set the drive motor target velocities.
     * <p>
     *     RunMode should e RUN_USING_ENCODER.
     * </p>
     * TODO: velocity is not abstract; it is ticksPerSecond
     * @param velocityLeft velocity for left motor (ticks/second)
     * @param velocityRight velocity for right motor (ticks/second)
     */
    static void setVelocity(double velocityLeft, double velocityRight) {
        // set the velocity for each motor
        dcmotorLeft.setVelocity(velocityLeft);
        dcmotorRight.setVelocity(velocityRight);
    }

    /**
     * Yuk. Want to zero the integration term in the PID controller.
     */
    static void resetPID() {
        dcmotorLeft.setMotorDisable();
        dcmotorRight.setMotorDisable();
        dcmotorLeft.setMotorEnable();
        dcmotorRight.setMotorEnable();
    }

    /**
     * Set the drive motor target velocity. (RunMode should be RUN_USING_ENCODER.)
     * @param velocity velocity for the motors (ticks/second)
     */
    static void setVelocity(double velocity) {
        // set the velocity
        setVelocity(velocity, velocity);
    }

    /**
     * Set the DcMotor mode for the drive motors.
     * @param runMode RunMode to use, e.g., RunMode.RUN_TO_POSITION.
     */
    static void setMode(DcMotor.RunMode runMode) {
        // set the mode for both motors
        dcmotorLeft.setMode(runMode);
        dcmotorRight.setMode(runMode);
    }

    /**
     * Set the drive motor PIDF coefficients for a particular mode.
     * @param runMode run mode to modify
     * @param pidfCoefficients the desired coefficients
     */
    static void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients pidfCoefficients) {
        // set the PIDF values for both motors
        dcmotorLeft.setPIDFCoefficients(runMode, pidfCoefficients);
        dcmotorRight.setPIDFCoefficients(runMode, pidfCoefficients);
    }

    /*
     * The team has several robots, but those robot are not identical.
     * We'd like to test code on each of those robots, so
     * we need to change values for motors, gear ratios, and dimensions.
     * These are the parameters for the 2019 robot.
     */

    /**
     * Set Robot Dims for Rover Ruckus robot
     */
    static void setRobotDims2018() {
        // set the wheel diameters to 90 mm
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        // measured wheel separation times a fudge factor
        distWheel =  (0.305 / 2) * (360.0 / 362.0);

        // ticks per wheel revolution
        // CoreHex motor... 4 ticks per revolutions
        // CoreHex motor... 1:72 gear ratio
        // REV specs also say 288 ticks per revolution
        ticksPerWheelRev = CORE_HEX_TICKS_PER_REV;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * Set Robot Dims for Sky Stone robot
     */
    static void setRobotDims2019() {
        // set the wheel diameters to 90 mm
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        // measured wheel separation times a fudge factor
        distWheel = (14.0 - (1.0/16.0)) * 0.0254 / 2;

        // ticks per wheel revolution
        ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * Set Robot Dims for Ultimate Goal robot
     */
    static void setRobotDims2020() {
        // set the wheel diameters to 90 mm
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        // measured wheel separation times a fudge factor
        distWheel = (0.365) * (1850.0/1800.0) / 2;

        // ticks per wheel revolution
        ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * Set Robot Dims for the Freight Frenzy robot.
     */
    static void setRobotDims2021() {
        // set the wheel diameters to 90 mm
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        // measured wheel separation times a fudge factor
        // the wheel separation shrunk to 8.25 inches
        // TODO: fudge is significant; also some drift to the right
        // TODO: significant disparity in turn left versus right. Wheels must be slipping.
        // distWheel = (5.0/3.45) * (8.25 * 0.0254) / 2;
        // put vise over wheels
        distWheel = (5.0 / 3.3) * (8.25 * 0.0254) / 2.0;

        // ticks per wheel revolution
        // The motor has a 20-tooth sprocket, and the wheel has a 15-tooth sprocket.
        // Also means set PIDF.F based upon the gear ratio
        ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1 * 15.0 / 20.0;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * Set Robot Dims for the Freight Frenzy robot.
     */
    static void setRobotDims2022() {
        // set the wheel diameters to 90 mm
        mWheelDiameterLeft = 0.090;
        mWheelDiameterRight = 0.090;

        // set the wheel half separation
        distWheel = (7.6 * 1.027 * 0.0254) / 2.0;

        // ticks per wheel revolution
        // The motor has a 45-tooth gear, and the wheel has a 90-tooth sprocket.
        ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1 * (90.0 / 45.0);

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * Set the important dimensions of the robot.
     * Depends on variable robot.
     */
    static void setRobotDims() {
        switch (robot) {
            case ROBOT_2018:
                setRobotDims2018();
                break;
            case ROBOT_2019:
                setRobotDims2019();
                break;
            case ROBOT_2020:
                setRobotDims2020();
                break;
            case ROBOT_2021:
                setRobotDims2021();
                break;
            case ROBOT_2022:
            default:
                setRobotDims2022();
                break;
        }
    }

    /**
     * Update the robot pose.
     * Call this at every update.
     * Uses small angle approximations.
     * See COS495-Odometry by Chris Clark, 2011,
     * <a href="https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf">https://www.cs.princeton.edu/courses/archive/fall11/cos495/COS495-Lecture5-Odometry.pdf</a>
     */
    static void updateRobotPose() {
        // several calculations are needed

        // get the current encoder positions
        int ticksLeft = dcmotorLeft.getCurrentPosition();
        int ticksRight = dcmotorRight.getCurrentPosition();

        // calculate change in encoder ticks from last time step
        int deltaTicksLeft = ticksLeft - cEncoderLeft;
        int deltaTicksRight = ticksRight - cEncoderRight;

        // save the new encoder positions for the next time around
        cEncoderLeft = ticksLeft;
        cEncoderRight = ticksRight;

        // calculate the distance the wheels moved
        double distL = deltaTicksLeft * distpertickLeft;
        double distR = deltaTicksRight * distpertickRight;

        // approximate the arc length as the average of the left and right arcs
        double ds = (distR + distL) / 2;
        // approximate the angular change as the difference in the arcs divided by wheel offset from
        // center of rotation.
        double deltaTheta = (distR - distL) / ( 2 * distWheel);

        // approximate the hypotenuse as just ds
        // approximate the average change in direction as one half the total angular change
        double dx = ds * Math.cos(thetaPose + 0.5 * deltaTheta);
        double dy = ds * Math.sin(thetaPose + 0.5 * deltaTheta);

        // update the current pose
        xPose = xPose + dx;
        yPose = yPose + dy;
        thetaPose = thetaPose + deltaTheta;

        // convert to inches and degrees
        xPoseInches = xPose / 0.0254;
        yPoseInches = yPose / 0.0254;
        thetaPoseDegrees = thetaPose * (180.0 / Math.PI);
    }

    /***
     * Set the current robot pose.
     * @param x robot x position in inches
     * @param y robot y position in inches
     * @param theta robot orientation in degrees
     */
    static void setPoseInches(double x, double y, double theta) {
        // convert inches to meters, degrees to radians, and set state variables
        xPose = x * 0.0254;
        yPose = y * 0.0254;
        thetaPose = theta * (Math.PI / 180.0);

        // copy to imperial to shadow state for consistency
        xPoseInches = x;
        yPoseInches = y;
        thetaPoseDegrees = theta;
    }

    /**
     * Get the current position tolerance in meters.
     * @return tolerance in meters
     */
    static double getMotorToleranceMeters() {
        return dcmotorLeft.getTargetPositionTolerance() * distpertickLeft;
    }

    /**
     * Get the current position tolerance in inches.
     * @return tolerance in inches
     */
    static double getMotorToleranceInches() {
        return getMotorToleranceMeters() / 0.0254 ;
    }

    /**
     * Set the position tolerance.
     * @param m tolerance in meters
     */
    static void setMotorToleranceMeters(double m) {
        int ticksLeft = (int)(m / distpertickLeft);
        int ticksRight = (int)(m / distpertickRight);

        dcmotorLeft.setTargetPositionTolerance(ticksLeft);
        dcmotorRight.setTargetPositionTolerance(ticksRight);
    }

    /**
     * Set the position tolerance.
     * @param inch tolerance in inches
     */
    static void setMotorToleranceInches(double inch) {
        // convert inches to meters and set the tolerance
        setMotorToleranceMeters(inch * 0.0254);
    }

    /**
     * Test if motors have reached their target.
     * @return true if both drive motors are close to target
     */
    static int fudgeCounter = 40;
    static boolean finished() {
        // if either motor is busy, return false
        if (dcmotorLeft.isBusy() ||  dcmotorRight.isBusy()) {
            // if stuff is busy, then reset the counter
            fudgeCounter = 20;
            return false;
        }
        else {
            // robot reached the position!

            if (fudgeCounter <= 0) {

                // log the position at the instant of success
                Log.d("Motion.finished()",
                        String.format("Pose (%.02f, %.02f) inches, heading %.01f degrees",
                                Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees));

                // return true
                return true;
            } else {
                fudgeCounter --;
                return false;
            }
        }
    }

    /**
     * Move the left and right motors a particular distance.
     * Commands the motors to move.
     * @param mLeft distance to move left motor in meters
     * @param mRight distance to move right motor in meters
     */
    static void moveMotorsMeters(double mLeft, double mRight) {
        // convert distance to motor ticks
        int ticksLeft = (int)(mLeft / distpertickLeft);
        int ticksRight = (int)(mRight / distpertickRight);

        dcmotorLeft.setTargetPosition(cEncoderLeft + ticksLeft);
        dcmotorRight.setTargetPosition(cEncoderRight + ticksRight);
    }

    /**
     * Move the left and right motors a particular distance.
     * Commands the motors to move.
     * @param inLeft distance to move the left motor in inches
     * @param inRight distance to move the right motor in inches
     */
    static void moveMotorsInches(double inLeft, double inRight) {
        moveMotorsMeters(inLeft * 0.0254, inRight * 0.0254);
    }

    static void moveMotorsTiles(double tileLeft, double tileRight) {
        moveMotorsMeters(tileLeft * metersPerTile, tileRight * metersPerTile);
    }

    /**
     * Move straight ahead a particular distance.
     * Commands the motors to move.
     * @param m distance in meters
     */
    static void moveMeters(double m) {
        moveMotorsMeters(m, m);
    }

    /**
     * Move straight ahead a particular distance.
     * Commands the motors to move.
     * @param in distance in inches
     */
    static void moveInches(double in) {
        // convert inches to meters and call moveMeters()
        moveMeters(in * 0.0254);
    }

    static void moveTiles(double tile) {
        moveMeters(tile * metersPerTile);
    }

    /**
     * Turn a relative angle in radians.
     * Commands the motors to move.
     * @param radians angle in radians
     */
    static void turnRadians(double radians) {
        // multiply by the radius in meters to get the circumferential distance
        double dist = radians * distWheel;
        // command the motors
        moveMotorsMeters(-dist, dist);
    }

    /**
     * Turn an angle in degrees.
     * Commands the motors to move.
     * @param degrees angle to turn in degrees
     */
    static void turnDegrees(double degrees) {
        // convert degrees to radians
        double radians = degrees * (Math.PI / 180.0);
        // command the turn
        turnRadians(radians);
    }

    /**
     * Calculate the heading from the current position to (x, y) in inches
     * @param x x coordinate in inches
     * @param y y coordinate in inches
     * @return heading angle in radians
     */
    static double headingInches(double x, double y) {
        double inchDX = x - xPoseInches;
        double inchDY = y - yPoseInches;

        return Math.atan2(inchDY, inchDX);
    }

    static double headingTiles(double x, double y) {
        double mDX = x * metersPerTile - xPose;
        double mDY = y * metersPerTile - yPose;

        return Math.atan2(mDY, mDX);
    }

    /**
     * Point the robot to the position (x, y) in inches.
     * Commands the motors to move.
     * @param x y coordinate in inches
     * @param y y coordinate in inches
     */
    static void headTowardInches(double x, double y) {
        // get the heading
        double heading = headingInches(x, y);
        // calculate the relative turn
        double radianTurn = AngleUnit.normalizeRadians(heading - thetaPose);

        // AngleUnit does not specify how the value is normalized!
        // Make sure it is a -PI to PI range
        if (radianTurn > Math.PI) radianTurn = radianTurn - 2 * Math.PI;

        // execute the turn
        turnRadians(radianTurn);
    }

    static void headTowardTiles(double x, double y) {
        // get the heading
        double heading = headingTiles(x, y);
        // calculate the relative turn
        double radianTurn = AngleUnit.normalizeRadians(heading - thetaPose);

        // AngleUnit does not specify how the value is normalized!
        // Make sure it is a -PI to PI range
        if (radianTurn > Math.PI) radianTurn = radianTurn - 2 * Math.PI;

        // execute the turn
        turnRadians(radianTurn);
    }

    /**
     * Compute the distance from the current position to (x,y)
     * @param x x-position in inches
     * @param y y-position in inches
     * @return distance in inches
     */
    static double distanceToInches(double x, double y) {
        // currently at (xPoseInches, yPoseInches)
        // d = sqrt((x1 -x2)^2 + (y1-y2)^2 )
        return Math.hypot(x-xPoseInches, y-yPoseInches);
    }

    /**
     * Find the distance in tiles to a position
     * @param xTiles position's x in tiles
     * @param yTiles position's y in tiles
     * @return distance in tiles
     */
    static double distanceToTiles(double xTiles, double yTiles) {
        double xMeters = xTiles * metersPerTile;
        double yMeters = yTiles * metersPerTile;

        // compute the distance in meters and then convert to tiles
        return Math.hypot(xMeters - xPose, yMeters - yPose) / metersPerTile;
    }

    static void reportPosition(Telemetry telemetry) {
        telemetry.addData("position",
                String.format((Locale)null, "(%6.01f %6.01f) inches, heading %6.01f degrees",
                        xPoseInches, yPoseInches, thetaPoseDegrees));
    }

}