package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

@Autonomous(name="Auto Score Low", group ="CodeDev")
public class AutoScoreLow extends OpMode {

    // the Vision object
    Vision vision;
    // the elevator
    Elevator elevator;

    // the gripper
    Gripper gripper;
    /** the LynxModule serial number */
    String strSerialNumber;

    enum State {
        STATE_START,
        STATE_TURN1,
        STATE_MOVE1,
        STATE_BACK,
        STATE_DROP,
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
        // create the vision object
        vision = new Vision();

        elevator = new Elevator(hardwareMap);

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

        // run using position
        Motion.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Motion.setPower(0.65);

        Motion.moveInches(Motion.distanceToInches(36.0, -36.0));
        gripper.grip(Gripper.GripState.GRIP_CLOSED);
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

        telemetry.addData("Motion Finished", Motion.finished());
        telemetry.addData("Elevator finished", elevator.finished());
        telemetry.addData("Elevator delta", (elevator.getTargetPosition() - elevator.getCurrentPosition()));

        if (gamepad1.y) {
            // set the pose
            Motion.setPoseInches(Vision.inchX, Vision.inchY, Vision.degTheta);
        }

        // dispatch on state
        switch (state) {
            case STATE_START:
                // wait for the gripper to close
                if (gripper.finished()){
                    // and then start raising the elevator
                    elevator.setTargetPosition(10.0);
                }
                // have we finished moving?
                if (Motion.finished()){
                    // then start heading toward the high junction
                    Motion.headTowardInches(48, -24);
                    // and start raising the elevator
                    elevator.setTargetPosition(Elevator.TargetPosition.LOW);
                    state = State.STATE_TURN1;
                }
                break;

            case STATE_TURN1:
                // we are turning toward the junction and raising the elevator
                if (Motion.finished() && elevator.finished()) {
                    // Move to center cone over the junction
                    Motion.moveInches(Motion.distanceToInches(48,-24)-10.5);

                    state = State.STATE_DROP;
                }
                break;

            case STATE_DROP:
                // when we get to the  junction
                if (Motion.finished()){
                    // open the gripper
                    gripper.grip(Gripper.GripState.GRIP_OPEN);

                    state = State.STATE_MOVE1;
                }
                break;

            case STATE_MOVE1:
                // wait for the gripper to open
                if (gripper.finished()){
                    // and then back up
                    Motion.moveInches(-5);

                    // and drop the elevator (while backing to minimize hance
                    elevator.setTargetPosition(Elevator.TargetPosition.FLOOR);

                    state= State.STATE_TURN2;
                }
                break;

            case STATE_TURN2:
                // finished backing up?
                if (Motion.finished()){
                    // turn more than needed to help the camera see.
                    Motion.headTowardInches(72, -48);
                    state = State.STATE_MOVE2;
                }
                break;
            case STATE_MOVE2:
                if (Motion.finished()){
                    // set pose if available...

                    // head toward the Zone 1, 2, 3
                    Motion.headTowardInches(72, -36);
                    state=State.STATE_MOVE3;
                }
                break;
            case STATE_MOVE3:
                // finished turning?
                if (Motion.finished()){
                    // move to Zone 1
                    // Motion.moveInches(-Motion.distanceToInches(12,-36));
                    // Move to Zone 2
                    // Motion.moveInches(-Motion.distanceToInches(36, -36));
                    // Move to Zone 3
                    Motion.moveInches(Motion.distanceToInches(60, -36));
                    state= State.STATE_TURN3;
                }
                break;
            case STATE_TURN3:
                // finished moving?
                if (Motion.finished()){
                    // Finish for Zone 1
                    // Motion.headTowardInches(12, -72);
                    // Finish for Zone 2
                    // Motion.headTowardInches(36, -72);
                    // Finish for Zone 3
                    Motion.headTowardInches(60, -72);
                    state = State.STATE_MOVE4;
                }
                break;
            case STATE_MOVE4:
                if (Motion.finished()){
                    // we be done?
                    Motion.moveInches(-5.0);
                    state= State.STATE_MOVE5;
                    state = State.STATE_END;
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