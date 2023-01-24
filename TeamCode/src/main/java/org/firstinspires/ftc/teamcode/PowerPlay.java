package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Class to manage some aspects of the PowerPlay game.
 */
public class PowerPlay {
    /** We can be on the BLUE or the RED alliance */
    enum Alliance {BLUE, RED}
    /** Our current alliance */
    public static Alliance alliance = Alliance.RED;

    /** We can start in the left or the right position */
    enum StartPos {LEFT, RIGHT}
    /** assume we start in the right position */
    public static StartPos startPos = StartPos.RIGHT;

    public static void init() {
        setPose();
    }

    /**
     * Called during the Autonomous init_loop.
     * Obtains configuration information from the gamepad
     * @param gamepad1
     */
    public static void init_loop(Telemetry telemetry, Gamepad gamepad1) {
        // set the alliance
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
            setPose();
        }
        if (gamepad1.b) {
            alliance = Alliance.RED;
            setPose();
        }

        // set the starting position
        if (gamepad1.dpad_left) {
            startPos = StartPos.LEFT;
            setPose();
        }
        if (gamepad1.dpad_right) {
            startPos = StartPos.RIGHT;
            setPose();
        }

        // possibly change starting conditions
        telemetry.addData("alliance: x (Blue) or b (Red)", alliance);
        telemetry.addData("startPos: dpad left or right", startPos);
    }


    /**
     * Set the robot pose based on alliance and starting position
     */
    public static void setPose() {
        double robotBackDistance = 7.75;
        double dx = 36.0;
        double fy = 70.75 - robotBackDistance;

        if (startPos == StartPos.LEFT) {
            if (alliance == Alliance.RED) {
                Motion.setPoseInches(-dx, -fy, 90.0);
            } else {
                Motion.setPoseInches(+dx, +fy, -90.0);
            }
        } else {
            // starting position is RIGHT
            if (alliance == Alliance.RED) {
                Motion.setPoseInches(+dx, -fy, +90.0);
            } else {
                Motion.setPoseInches(-dx, +fy, -90.0);
            }
        }
    }

}
