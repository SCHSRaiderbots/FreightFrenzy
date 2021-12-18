package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A static class used to set the game conditions.
 * Need to set the Alliance, starting position, and ending position.
 */
public class GameConfig {
    // define the autonomous options

    /**
     * Possible Alliances: Red or Blue
     */
    public enum Alliance {
        RED(1.0), BLUE(-1.0);

        /**
         * Y axis scale (used to mirror the Y axis according to the alliance)
         */
        final double yScale;

        Alliance(double scale) {
            this.yScale = scale;
        }

        /**
         * Get the next enum...
         * @return next enum
         */
        Alliance next() {
            return (this == RED)? BLUE : RED;
        }
    }

    /**
     * Possible starting locations for the robot
     */
    public enum LocationStart {
        AUDIENCE(-50.0, -72.0),
        WAREHOUSE(-10.0, -72.0);

        /** Robot starting x-position (inches) */
        final double x;
        /** Robot starting y-position (inches) */
        final double y;

        /** Initialize a starting location */
        LocationStart(double x, double y) {
            this.x = x;
            this.y = y;
        }

        /**
         * Get the next enum
         * @return next enum
         */
        LocationStart next() {
            // this enum's index
            int ord = this.ordinal();
            // array of values
            LocationStart[] values = LocationStart.values();

            // wrap around increment
            return values[(ord+1) % values.length];
        }
    }

    /**
     * Possible ending locations for the robot
     */
    public enum LocationEnd {
        WAREHOUSE_INNER(50.0, -20.0),
        WAREHOUSE_OUTER(50.0, -40.0),
        STORAGE_UNIT(-60.0, -40.0);

        /** robot ending x-position (inches) */
        final double x;
        /** robot ending y-position (inches) */
        final double y;

        /** initialize an ending location */
        LocationEnd(double x, double y) {
            this.x = x;
            this.y = y;
        }

        /**
         * Get the next enum
         * @return next enum
         */
        LocationEnd next() {
            // this enum's index
            int ord = this.ordinal();
            // array of values
            LocationEnd[] values = LocationEnd.values();

            // wrap around increment
            return values[(ord+1) % values.length];
        }
    }

    // set the default options
    // These are static and should continue across op mode selections
    public static Alliance alliance = Alliance.RED;
    public static LocationStart locationStart = LocationStart.AUDIENCE;
    public static LocationEnd locationEnd = LocationEnd.STORAGE_UNIT;

    // remember the game pad state so we can detect transitions
    static private boolean bAlliance = false;
    static private boolean bLocationStart = false;
    static private boolean bLocationEnd = false;

    /**
     * Initialize the Game Configuration Dialog.
     * Call during the Autonomous OpMode init() routine
     */
    static void init() {
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
        telemetry.addData("Enum data", "scale %4.0f, start (%6.01f, %6.01f), end (%6.01f, %6.01f)",
                alliance.yScale,
                locationStart.x, alliance.yScale * locationStart.y,
                locationEnd.x, alliance.yScale * locationEnd.y);
    }
}
