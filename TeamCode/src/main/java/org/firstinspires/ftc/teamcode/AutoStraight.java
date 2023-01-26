package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;
import static org.firstinspires.ftc.teamcode.Motion.setVelocity;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

@Autonomous(name="Auto Straight", group ="CodeDev")
public class AutoStraight extends OpMode {
    // the elevator
    Elevator elevator;

    // the gripper
    Gripper gripper;

    // the Vision object
    Vision vision;

    /** the LynxModule serial number */
    String strSerialNumber;

    enum State {
        STATE_START,
        STATE_TURN1,
        STATE_MOVE1,
        STATE_BACK,
        STATE_TURN2,
        STATE_MOVE2,
        STATE_TURN3,
        STATE_MOVE3,
        STATE_TURN4,
        STATE_MOVE4,
        STATE_MOVE5,
        STATE_MOVE6,
        STATE_END,
        STATE_PARK
    }
    State state= State.STATE_START;

    @Override
    public void init() {
        // set the robot
        // Motion.identifyRobot(hardwareMap);
        robot = Motion.Robot.ROBOT_2022;

        // initialize motion
        Motion.init(hardwareMap);

        // set the initial position
        PowerPlay.init();

        // the elevator
        elevator = new Elevator(hardwareMap);

        // the gripper
        gripper = new Gripper(hardwareMap);
        // make sure the gripper is open
        gripper.grip(Gripper.GripState.GRIP_OPEN);

        // create the vision object
        vision = new Vision();

        // init the Vuforia localization engine
        vision.initVuforia(hardwareMap);

        // init tracking
        vision.initTracking();

        // build an object detector
        vision.initTfod(hardwareMap);

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
        PowerPlay.init_loop(telemetry, gamepad1);
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

        // close the gripper
        gripper.grip(Gripper.GripState.GRIP_CLOSED);

        // run using position
        // Motion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Motion.setPower(0.65);

        Motion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Motion.setPower(0.65);

        Motion.setVelocity(600,600);

        state=State.STATE_START;
    }

    @Override
    public void loop() {
        // update the robot pose
        Motion.updateRobotPose();
        Motion.reportPosition(telemetry);

        // report targets in view
        vision.reportTracking(telemetry);

        telemetry.addData("State", state.toString());

        // do some driving
        // double forward = -0.7 * gamepad1.left_stick_y;
        // double turn = 0.4 * gamepad1.right_stick_x;

        switch (state) {
            case STATE_START:
            {
                // distance to target y position
                double dist = Motion.yPoseInches - -12.0;

                telemetry.addData("dist", dist);

                // positive distance means we got there....
                if (dist > 0.0) {
                    Motion.setVelocity(0,0);

                    Motion.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Motion.setPower(0.4);

                    // This is the junction
                    Motion.headTowardInches(24, 0);

                    state= State.STATE_TURN1;
                }
            }

                // check if the gripper is now closed...
                if (gripper.finished()) {
                    // start moving the elevator
                    elevator.setTargetPosition(10.0);
                }

                if (Motion.finished()){
                    // Motion.headTowardInches(24, 0);
                    // state = State.STATE_TURN1;
                }
                break;

            case STATE_TURN1:
                if (Motion.finished()) {
                    // finished turning.
                    // move distance to junction less a bit for the robot.
                    Motion.moveInches(Motion.distanceToInches(24,0)-9);
                    state= State.STATE_MOVE1;
                }
                break;

            case STATE_MOVE1:
                if (Motion.finished()){
                    // really want to back up until y is -12.
                    Motion.moveInches(-Motion.distanceToInches(36,-12));
                    state= State.STATE_TURN2;
                }
                break;
            case STATE_TURN2:
                if (Motion.finished()){
                    Motion.headTowardInches(72, -12);
                    state = State.STATE_MOVE2;
                }
                break;
            case STATE_MOVE2:
                if (Motion.finished()){
                    Motion.moveInches(Motion.distanceToInches(60,-12));
                    state=State.STATE_MOVE3;
                }
                break;
            case STATE_MOVE3:
                if (Motion.finished()){
                    Motion.moveInches(-Motion.distanceToInches(12,-12));
                    state= State.STATE_TURN3;
                }
                break;
            case STATE_TURN3:
                if (Motion.finished()){
                    Motion.headTowardInches(0, -24);
                    state = State.STATE_MOVE4;
                }
                break;
            case STATE_MOVE4:
                if (Motion.finished()){
                    Motion.moveInches(Motion.distanceToInches(0,-24)-9);
                    state= State.STATE_MOVE5;
                }
                break;
            case STATE_MOVE5:
                if (Motion.finished()){
                    Motion.moveInches(-Motion.distanceToInches(0,-24));
                    state= State.STATE_TURN4;
                }
                break;
            case STATE_TURN4:
                if (Motion.finished()){
                    Motion.headTowardInches(72, -12);
                    state = State.STATE_MOVE6;
                }
                break;
            case STATE_MOVE6:
                if (Motion.finished()){
                    Motion.moveInches(Motion.distanceToInches(60,-12));
                    state= State.STATE_END;
                }
                break;
            case STATE_END:
                if (Motion.finished()){

                }
                break;


        }





        // Motion.setPower(forward+turn, forward-turn);
    }

    @Override
    public void stop() {
        // turn off tracking
        vision.targets.deactivate();
    }
}