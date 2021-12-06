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
    public enum StartLocation {AUDIENCE, WAREHOUSE}
    public enum EndLocation {WAREHOUSE_INNER, WAREHOUSE_OUTER, STORAGE_UNIT}

    // set the default options
    // These are static and should continue across op mode selections
    public static Alliance alliance = Alliance.RED;
    public static StartLocation startLocation = StartLocation.AUDIENCE;
    public static EndLocation endLocation = EndLocation.WAREHOUSE_INNER;

    // remember gamepad state so we can detect transitions
    static private boolean bAlliance = false;
    static private boolean bStartLocation = false;
    static private boolean bEndLocation = false;

    static void init(Gamepad gamepad1) {
        bAlliance=false;
        bStartLocation = false;
        bEndLocation = false;
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

        // gamepad1.b sets the StartLocation
        if (gamepad1.b) {
            // b button pressed
            if (!bStartLocation) {
                startLocation = (startLocation == StartLocation.AUDIENCE) ? StartLocation.WAREHOUSE : StartLocation.AUDIENCE;
                bStartLocation = true;
            }
        } else {
            // b button not pressed
            bStartLocation = false;
        }

        // gamepad1.x sets the EndLocation
        if (gamepad1.x) {
            // x button is pressed
            if (!bEndLocation) {
                switch (endLocation) {
                    case WAREHOUSE_INNER:
                        endLocation = EndLocation.WAREHOUSE_OUTER;
                        break;
                    case WAREHOUSE_OUTER:
                        endLocation = EndLocation.STORAGE_UNIT;
                        break;
                    default:
                    case STORAGE_UNIT:
                        endLocation = EndLocation.WAREHOUSE_INNER;
                        break;
                }
                // button was pressed
                bEndLocation = true;
            }
        } else {
            // x button is not pressed
            bEndLocation = false;
        }
    }

    static void report(Telemetry telemetry) {
        // report the configuration
        telemetry.addData("Alliance  (a)", alliance);
        telemetry.addData("Start Loc (b)", startLocation);
        telemetry.addData("End Loc   (x)", endLocation);
    }
}
