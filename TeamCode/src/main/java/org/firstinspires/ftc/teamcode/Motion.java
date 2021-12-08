package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

/**
 * This class consolidates some of the robot motion calculations.
 * It uses static methods because there is only one physical robot.
 * Furthermore, we want to carry the robot's current position from one OpMode to the next.
 * For example, an Autonomous routine might set a particular starting position.
 * At the end of the Autonomous routine, the robot will have an updated position.
 * By using static variables, the succeeding Teleop routine will know the robot's position.
 *
 * This code is borrowing from 2019 and 2020 robot code
 *
 * So the OpMode should:
 * tell this class which motors are being used:
 *   Motion.setRobot(dcmotorLeft, dcmotorRight);
 * tell this class the robot dimensions:
 *   Motion.setRobot2019();
 * and then make periodic calls to update the robot position:
 *   Motion.updateRobotPose()
 * the current position may be accessed with
 *   Motion.xPose, Motion.yPose, Motion.thetaPose
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

    // The HD Hex Motor is also used with the Ultraplanetary cartridges.
    // These values are used to calculate actual gear ratios
    //    the 3:1 cartridge is actually 84:29 (2.9...)
    //    the 4:1 cartridge is actually 76:21 (3.6...)
    //    the 5:1 cartridge is actually 68:13 (5.2...)
    public static final double HD_HEX_GEAR_CART_3_1 = 84.0/29.0;
    public static final double HD_HEX_GEAR_CART_4_1 = 76.0/21.0;
    public static final double HD_HEX_GEAR_CART_5_1 = 68.0/13.0;

    // Info about the actual robot

    // robot parameters

    // the wheel diameters are 90mm nominal
    static private double mWheelDiameterLeft = 0.090;
    static private double mWheelDiameterRight = 0.090;

    // half the distance between the wheels
    // the new wheel separation 13 + 15/16
    static double distWheel = (14.0 - (1.0/16.0)) * 0.0254 / 2;

    // calculate the wheel's ticks per revolution
    static double ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

    // derived robot parameters
    // Distance per tick
    //   leaving the units vague at this point

    // the distance per tick for each wheel = circumference / ticks
    static private double distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
    static private double distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);


    // the robot pose
    //   can have .updatePose(), .getPose(), .setPose()
    //   using static should allow the Pose to be carried over from Autonomous to Teleop
    //     Autonomous can set the initial pose
    //     When Teleop starts, it can use the existing Pose
    //        If there was no autonomous, then initial Pose is random
    //        A button press during teleop's init_loop could set a known Pose
    // these values are in meters
    static double xPose = 0.0;
    static double yPose = 0.0;
    // angle is in radians
    static double thetaPose = 0.0;

    // TODO: use better plan for values that must stay in sync
    static double xPoseInches = 0.0;
    static double yPoseInches = 0.0;
    static double thetaPoseDegrees = 0.0;

    static private DcMotorEx dcmotorLeft;
    static private DcMotorEx dcmotorRight;

    // encoder counts
    // There's a subtle issue here
    //    If robot is not moving, it is OK to set these values to the current encoder counts
    //    That could always happen during .init()
    static private int cEncoderLeft;
    static private int cEncoderRight;

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
        distWheel = (14.0 - (1.0/16.0)) * 0.0254 / 2;

        // ticks per wheel revolution
        ticksPerWheelRev = HD_HEX_TICKS_PER_REV * HD_HEX_GEAR_CART_5_1 * HD_HEX_GEAR_CART_4_1;

        // derived values
        distpertickLeft = mWheelDiameterLeft * Math.PI / (ticksPerWheelRev);
        distpertickRight = mWheelDiameterRight * Math.PI / (ticksPerWheelRev);
    }

    /**
     * Set Robot Dims for the Freight Frenzy robot
     */
    static void setRobotDims2021() {
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
        int dticksLeft = ticksLeft - cEncoderLeft;
        int dticksRight = ticksRight - cEncoderRight;

        // save the new encoder positions for the next time around
        cEncoderLeft = ticksLeft;
        cEncoderRight = ticksRight;

        // calculate the distance the wheels moved
        double distL = dticksLeft * distpertickLeft;
        double distR = dticksRight * distpertickRight;

        // approximate the arc length as the average of the left and right arcs
        double ds = (distR + distL) / 2;
        // approximate the angular change as the difference in the arcs divided by wheel offset from
        // center of rotation.
        double dtheta = (distR - distL) / ( 2 * distWheel);

        // approximate the hypotenuse as just ds
        // approximate the average change in direction as one half the total angular change
        double dx = ds * Math.cos(thetaPose + 0.5 * dtheta);
        double dy = ds * Math.sin(thetaPose + 0.5 * dtheta);

        // update the current pose
        xPose = xPose + dx;
        yPose = yPose + dy;
        thetaPose = thetaPose + dtheta;

        // convert to inches and degrees
        xPoseInches = xPose / 0.0254;
        yPoseInches = yPose / 0.0254;
        thetaPoseDegrees = thetaPose * (180.0 / Math.PI);
    }

    /***
     * Set the current robot pose
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

    static double getMotorToleranceMeters() {
        return dcmotorLeft.getTargetPositionTolerance() * distpertickLeft;
    }

    static double getMotorToleranceInches() {
        return getMotorToleranceMeters() / 0.0254 ;
    }

    static void setMotorToleranceMeters(double m) {
        int ticksLeft = (int)(m / distpertickLeft);
        int ticksRight = (int)(m / distpertickRight);

        dcmotorLeft.setTargetPositionTolerance(ticksLeft);
        dcmotorRight.setTargetPositionTolerance(ticksRight);
    }

    static void setMotorToleranceInches(double inch) {
        // convert inches to meters and set the tolerance
        setMotorToleranceMeters(inch * 0.0254);
    }

    /**
     * Move the left and right motors a particular distance
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
     * Move the left and right motors a particular distance
     * @param inLeft distance to move the left motor in inches
     * @param inRight distance to move the right motor in inches
     */
    static void moveMotorsInches(double inLeft, double inRight) {
        moveMotorsMeters(inLeft * 0.0254, inRight * 0.0254);
    }

    /**
     * Move straight ahead a particular distance
     * @param m distance in meters
     */
    static void moveMeters(double m) {
        moveMotorsMeters(m, m);
    }

    /**
     * Move straight ahead a particular distance
     * @param in distance in inches
     */
    static void moveInches(double in) {
        // convert inches to meters and call moveMeters()
        moveMeters(in * 0.0254);
    }

    /**
     * Turn a relative angle
     * @param radians angle in radians
     */
    static void turnRadians(double radians) {
        // multiply by the radius in meters to get the circumferential distance
        double dist = radians * distWheel;
        // command the motors
        moveMotorsMeters(dist, -dist);
    }

    /**
     * Turn an angle in degrees
     * @param degrees angle to turn in degrees
     */
    static void turn(double degrees) {
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

    /**
     * Point the robot to the position (x, y) in inches
     * @param x y coordinate in inches
     * @param y y coordinate in inches
     */
    static void headTowardInches(double x, double y) {
        // get the heading
        double heading = headingInches(x, y);
        // calculate the relative turn
        double radianTurn = heading - thetaPose;
        // execute the turn
        turnRadians(radianTurn);
    }

}