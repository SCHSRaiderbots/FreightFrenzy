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
    public enum Alliance {RED, BLUE}
    public enum LocationStart {AUDIENCE, WAREHOUSE}
    public enum LocationEnd {WAREHOUSE_INNER, WAREHOUSE_OUTER, STORAGE_UNIT}

    // set the default options
    // These are static and should continue across op mode selections
    public static Alliance alliance = Alliance.RED;
    public static LocationStart locationStart = LocationStart.AUDIENCE;
    public static LocationEnd locationEnd = LocationEnd.WAREHOUSE_INNER;

    // remember gamepad state so we can detect transitions
    static private boolean bAlliance = false;
    static private boolean bLocationStart = false;
    static private boolean bLocationEnd = false;

    static void init(Gamepad gamepad1) {
        bAlliance=false;
        bLocationStart = false;
        bLocationEnd = false;
    }

    static void init_loop(Gamepad gamepad1) {
        // gamepad1.a sets the Alliance
        if (gamepad1.a) {
            // a button pressed
            if (!bAlliance) {
                alliance = (alliance == Alliance.RED) ? Alliance.BLUE : Alliance.RED;
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
                locationStart = (locationStart == LocationStart.AUDIENCE) ? LocationStart.WAREHOUSE : LocationStart.AUDIENCE;
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
                switch (locationEnd) {
                    case WAREHOUSE_INNER:
                        locationEnd = LocationEnd.WAREHOUSE_OUTER;
                        break;
                    case WAREHOUSE_OUTER:
                        locationEnd = LocationEnd.STORAGE_UNIT;
                        break;
                    default:
                    case STORAGE_UNIT:
                        locationEnd = LocationEnd.WAREHOUSE_INNER;
                        break;
                }
                // button was pressed
                bLocationEnd = true;
            }
        } else {
            // x button is not pressed
            bLocationEnd = false;
        }
    }

    static void report(Telemetry telemetry) {
        // report the configuration
        telemetry.addData("Alliance  (a)", alliance);
        telemetry.addData("Start Loc (b)", locationStart);
        telemetry.addData("End Loc   (x)", locationEnd);
    }
}
