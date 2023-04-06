package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;
import static org.firstinspires.ftc.teamcode.Vision.Signal.SIGNAL1;
import static org.firstinspires.ftc.teamcode.Vision.Signal.SIGNAL3;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

@Autonomous(name="Auto Score High", group ="CodeDev")
public class AutoScoreHigh extends OpMode {

    // the Vision object
    Vision vision;
    // the elevator
    Elevator elevator;

    // the gripper
    Gripper gripper;

    enum State {
        STATE_START,
        STATE_SECOND,
        STATE_TURN1,
        STATE_MOVE1,
        STATE_DROP,
        STATE_TURN2,
        STATE_MOVE2,
        STATE_TURN3,
        STATE_MOVE3,
        STATE_MOVE4,
        STATE_READ_NAV,
        STATE_END
    }
    State state= State.STATE_START;

    // mapping routines
    double xMap (double tileX) {
        if (PowerPlay.alliance == PowerPlay.Alliance.RED && PowerPlay.startPos == PowerPlay.StartPos.RIGHT ||
                PowerPlay.alliance == PowerPlay.Alliance.BLUE && PowerPlay.startPos == PowerPlay.StartPos.LEFT) {
            return tileX;
        } else {
            return -tileX;
        }
    }

    double yMap (double tileY) {
        if (PowerPlay.alliance == PowerPlay.Alliance.RED) {
            return tileY;
        } else {
            return -tileY;
        }
    }

    void MappedHeadToward(double tileX, double tileY) {
        // figure the mapped position
        double tx = xMap(tileX);
        double ty = yMap(tileY);

        // head toward that position
        Motion.headTowardTiles(tx, ty);
    }

    double MappedDistance(double tileX, double tileY) {
        // figure the mapped position in tiles
        double tx = xMap(tileX);
        double ty = yMap(tileY);

        return Motion.distanceToTiles(tx, ty);
    }

    @Override
    public void init() {
        // create the vision object
        vision = new Vision();

        elevator = new Elevator(hardwareMap, true);

        // init the Vuforia localization engine
        vision.initVuforia(hardwareMap);

        // init tracking
        vision.initTracking();

        // build an object detector
        vision.initTfod(hardwareMap);
        // the gripper
        gripper = new Gripper(hardwareMap);
        // make sure the gripper is open
        gripper.grip(Gripper.GripState.GRIP_OPEN);

        // if we have an object detector, then activate it
        if (vision.tfod != null) {
            vision.tfod.activate();
            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            // vision.tfod.setZoom(1.25, 16.0 / 9.0);
        }

        // initialize motion
        // Motion.identifyRobot(hardwareMap);
        robot = Motion.Robot.ROBOT_2022;
        Motion.init(hardwareMap);

        // run using position
        Motion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // at 0.65, I slip (even with water bottle)
        // at 0.60, a little slipping
        // at 0.50, 15 seconds left for park at Zone 2
        // at 0.45, 12 seconds (Motion.finished() delay reduced to 20 calls)
        // at 0.40, 11 seconds left for park at Zone 3
        // at 0.30, 06 seconds left for park at Zone 3
        // at 0.10, do not finish
        // turning left at 0.45 is not so good, so try less
        Motion.setPower(0.35);

        // set the initial position
        PowerPlay.init();
    }

    @Override
    public void init_loop() {
        // report the robot
        telemetry.addData("Robot", robot);

        // update the robot pose
        Motion.updateRobotPose();

        // report our position
        Motion.reportPosition(telemetry);

        // report detected objects
        // vision.reportDetections(telemetry);
        vision.readSignal();
        telemetry.addData("Signal", vision.signal);

        // set the alliance and start position
        // position is only set if there are DPAD changes
        PowerPlay.init_loop(telemetry, gamepad1);

        // TODO: Debugging
        telemetry.addData("x,y position", "(%6.3f, %6.3f)", xMap(1.5), yMap(-1.5));
        telemetry.addData("distance", MappedDistance(1.5, -1.5));

        if (gamepad2.dpad_right) {
            if (!Motion.finished()) {
                Motion.turnDegrees(-90.0);
            }
        }
        if (gamepad2.dpad_left) {
            if (!Motion.finished()) {
                Motion.turnDegrees(90.0);
            }
        }
        if (gamepad2.dpad_up) {
            if (!Motion.finished()) {
                Motion.moveInches(24.0);
            }
        }
        if (gamepad2.dpad_down) {
            if (!Motion.finished()) {
                Motion.moveInches(-24.0);
            }
        }
    }

    @Override
    public void start() {
        // report current status
        telemetry.addData("Navigation", "On");

        // we are no longer interested in object detection
        vision.tfod.deactivate();

        // we are interested in navigation targets
        if (vision.targets == null) {
            Log.d("vision.targets", "is null!");
        } else {
            vision.targets.activate();
        }

        // The SIGNAL is not symmetric.
        // As a HACK, map the signal when on the Left side
        if (PowerPlay.startPos == PowerPlay.StartPos.LEFT) {
            switch (vision.signal) {
                case SIGNAL1:
                    vision.signal = SIGNAL3;
                    break;
                case SIGNAL3:
                    vision.signal = SIGNAL1;
            }
        }

        // and make sure we are in the starting state
        state = State.STATE_START;
    }

    @Override
    public void loop() {
        // report the state
        telemetry.addData("state", state);

        // update the robot pose
        Motion.updateRobotPose();
        Motion.reportPosition(telemetry);

        // report targets in view
        vision.reportTracking(telemetry);

        // report status of the two subsystems
        telemetry.addData("Motion Finished", Motion.finished());
        telemetry.addData("Elevator finished", elevator.finished());

        // TODO: remove this
        if (gamepad1.y) {
            // set the pose
            Motion.setPoseInches(Vision.inchX, Vision.inchY, Vision.degTheta);
        }

        // dispatch on state
        switch (state) {
            case STATE_START:
                // The initial move...

                // Start moving the robot.
                Motion.moveTiles(MappedDistance(1.5, -0.5));

                // close the gripper to grab the cone
                gripper.grip(Gripper.GripState.GRIP_CLOSED);

                state = State.STATE_SECOND;
                break;

            case STATE_SECOND:
                // we are moving to the first waypoint
                // wait for the gripper to close, thn start lifting...
                if (gripper.finished()){
                    // and then start raising the elevator
                    elevator.setTargetPosition(10.0);
                }
                // have we finished moving?
                if (Motion.finished()){
                    // then start heading toward the target junction
                    MappedHeadToward(1.0, 0.0);

                    // and start raising the elevator for that junction
                    elevator.setTargetPosition(Elevator.TargetPosition.HIGH);
                    state = State.STATE_TURN1;
                }
                break;

            case STATE_TURN1:
                // we are turning toward the target junction and raising the elevator
                if (Motion.finished() && elevator.finished()) {
                    // Move to center cone over the junction
                    // TODO: Use a variable for the distance
                    Motion.moveTiles(MappedDistance(1.0, 0.0) - (10.5/24.0));

                    state = State.STATE_DROP;
                }
                break;

            case STATE_DROP:
                // we are approaching the junction.
                if (Motion.finished()){
                    // open the gripper
                    gripper.grip(Gripper.GripState.GRIP_OPEN);

                    state = State.STATE_MOVE1;
                }
                break;

            case STATE_MOVE1:
                // waiting for the gripper to open and cone to drop
                if (gripper.finished()){
                    // back up to get away from junction
                    Motion.moveInches(-5);

                    // drop the elevator (while backing to minimize gripper dropping on terminal)
                    elevator.setTargetPosition(Elevator.TargetPosition.FLOOR);

                    state= State.STATE_TURN2;
                }
                break;

            case STATE_TURN2:
                // we are backing up
                if (Motion.finished()){
                    // turn more than needed to help the camera see.
                    double deltaTurn = (PowerPlay.startPos == PowerPlay.StartPos.RIGHT) ? -0.5 : 0.5;
                    MappedHeadToward(3.0, -1.5 + deltaTurn);
                    state = State.STATE_MOVE2;
                }
                break;

            case STATE_MOVE2:
                // We are pointing the robot so we can read the Navigation Target
                if (Motion.finished()){
                    // set pose if available...

                    state=State.STATE_READ_NAV;
                }
                break;

            case STATE_READ_NAV:
                // we are trying to read the navigation target

                // straighten out to head toward the Zone 1, 2, 3
                MappedHeadToward(3.0, -0.5);

                // park in the appropriate Zone
                state = State.STATE_MOVE3;
                break;

            case STATE_MOVE3:
                // We are turning to be perpendicular to the wall
                if (Motion.finished()){

                    // now move to the appropriate x position for the Zone
                    switch (vision.signal) {
                        case SIGNAL1:
                            // Motion.moveInches(-Motion.distanceToInches(12,-36));
                            Motion.moveTiles(-MappedDistance(0.5, -0.5));
                            break;
                        case SIGNAL2:
                            // Motion.moveInches(-Motion.distanceToInches(36, -36));
                            Motion.moveTiles(-MappedDistance(1.5, -0.5));
                            break;
                        case SIGNAL3:
                            // Motion.moveInches(Motion.distanceToInches(60, -36));
                            Motion.moveTiles(MappedDistance(2.5, -0.5));
                            break;
                    }
                    state= State.STATE_TURN3;
                }
                break;

            case STATE_TURN3:
                // finished moving to Zone x position?
                if (Motion.finished()){
                    switch (vision.signal) {
                        // turn to the alliance wall
                        case SIGNAL1:
                            // Motion.headTowardInches(12, -72);
                            MappedHeadToward(0.5, -3.0);
                            break;
                        case SIGNAL2:
                            // Motion.headTowardInches(36, -72);
                            MappedHeadToward(1.5, -3.0);
                            break;
                        case SIGNAL3:
                            // Motion.headTowardInches(60, -72);
                            MappedHeadToward(2.5, -3.0);
                            break;
                    }
                    state = State.STATE_MOVE4;
                }
                break;

            case STATE_MOVE4:
                // we are turning to the alliance wall
                if (Motion.finished()){
                    // Now move forware a bit so we are in the Zone
                    Motion.moveInches(5.0);

                    state = State.STATE_END;
                }
                break;

            case STATE_END:
                // do nothing
                break;
        }
    }

    @Override
    public void stop() {
        // turn off tracking
        vision.targets.deactivate();
    }
}
