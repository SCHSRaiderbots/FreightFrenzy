package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A static class used to set the game conditions.
 * Need to set the Alliance, starting position, and ending position
 */
public class GameConfig {
    // define the autonomous options
    // we could put properties on the locations
    public enum Alliance {
        RED, BLUE;

        /**
         * Get the next enum...
         * @return next enum
         */
        Alliance next() {
            return (this == RED)? BLUE : RED;
        }
    }
    public enum LocationStart {
        AUDIENCE, WAREHOUSE;

        /**
         * Get the next enum
         * @return next enum
         */
        LocationStart next() {
            return (this == AUDIENCE) ? WAREHOUSE : AUDIENCE;
        }
    }
    public enum LocationEnd {
        WAREHOUSE_INNER, WAREHOUSE_OUTER, STORAGE_UNIT;

        /**
         * Get the next enum
         * @return next enum
         */
        LocationEnd next() {
            switch (this) {
                case WAREHOUSE_INNER:
                    return WAREHOUSE_INNER;
                case WAREHOUSE_OUTER:
                    return STORAGE_UNIT;
                default:
                case STORAGE_UNIT:
                    return WAREHOUSE_INNER;
            }
        }
    }

    // set the default options
    // These are static and should continue across op mode selections
    public static Alliance alliance = Alliance.RED;
    public static LocationStart locationStart = LocationStart.AUDIENCE;
    public static LocationEnd locationEnd = LocationEnd.WAREHOUSE_INNER;

    // remember gamepad state so we can detect transitions
    static private boolean bAlliance = false;
    static private boolean bLocationStart = false;
    static private boolean bLocationEnd = false;

    /**
     * Initialize the Game Configuration Dialog.
     * Call during the Autonomous OpMode init() routine
     * @param gamepad1 gamepad controlling configuration
     */
    static void init(Gamepad gamepad1) {
        bAlliance=false;
        bLocationStart = false;
        bLocationEnd = false;
    }

    /**
     * Process the Game Configuration Dialog messages.
     * Looks for gamepad buttons and updates the configuration.
     * @param gamepad1 gamepad controlling configuration
     */
    static void init_loop(Gamepad gamepad1) {
        // gamepad1.a sets the Alliance
        if (gamepad1.a) {
            // a button pressed
            if (!bAlliance) {
                alliance = alliance.next();
                bAlliance = true;
            }
        } else {
            // a button not pressed
            bAlliance = false;
        }

        // gamepad1.b sets the Starting Location
        if (gamepad1.b) {
            // b button pressed
            if (!bLocationStart) {
                locationStart = locationStart.next();
                bLocationStart = true;
            }
        } else {
            // b button not pressed
            bLocationStart = false;
        }

        // gamepad1.x sets the Ending Location
        if (gamepad1.x) {
            // x button is pressed
            if (!bLocationEnd) {
                locationEnd = locationEnd.next();
                // button was pressed
                bLocationEnd = true;
            }
        } else {
            // x button is not pressed
            bLocationEnd = false;
        }
    }

    /**
     * Report the game configuration on the Driver Station
     * @param telemetry telemetry object to transmit messages to Driver Station
     */
    static void report(Telemetry telemetry) {
        // report the configuration
        telemetry.addData("Alliance  (a)", alliance);
        telemetry.addData("Start Loc (b)", locationStart);
        telemetry.addData("End Loc   (x)", locationEnd);
    }
}
