package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Motion.robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import android.util.Log;

/**
 * This OpMode illustrates using the Vuforia localizer to determine positioning and orientation of
 * robot on the FTC field using a WEBCAM.  The code is structured as a LinearOpMode
 *
 * NOTE: If you are running on a Phone with a built-in camera, use the ConceptVuforiaFieldNavigation example instead of this one.
 * NOTE: It is possible to switch between multiple WebCams (eg: one for the left side and one for the right).
 *       For a related example of how to do this, see ConceptTensorFlowObjectDetectionSwitchableCameras
 *
 * When images are located, Vuforia is able to determine the position and orientation of the
 * image relative to the camera.  This sample code then combines that information with a
 * knowledge of where the target images are on the field, to determine the location of the camera.
 *
 * Finally, the location of the camera on the robot is used to determine the
 * robot's location and orientation on the field.
 *
 * To learn more about the FTC field coordinate model, see FTC_FieldCoordinateSystemDefinition.pdf in this folder
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */

@Autonomous(name="Auto Park", group ="CodeDev")
public class AutoPark extends OpMode {

    // the Vision object
    Vision vision;

    /** the LynxModule serial number */
    String strSerialNumber;

    /** We can be on the BLUE or the RED alliance */
    enum Alliance {BLUE, RED}
    /** Our current alliance */
    Alliance alliance = Alliance.RED;

    /** We can start in the left or the right position */
    enum StartPos {LEFT, RIGHT}
    /** assume we start in the right position */
    StartPos startPos = StartPos.RIGHT;
    enum State {
        STATE_START,
        STATE_MOVE,
        STATE_TURN,
        STATE_BACK,
        STATE_PARK
    }
    State state= State.STATE_START;



    @Override
    public void init() {
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

        // initialize motion
        // Motion.identifyRobot(hardwareMap);
        robot = Motion.Robot.ROBOT_2022;
        Motion.init(hardwareMap);

        // set the initial position
        setPose();
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

        Motion.setPower(0.5);

        Motion.moveInches(51);



    }

    @Override
    public void loop() {
        // update the robot pose
        Motion.updateRobotPose();
        Motion.reportPosition(telemetry);

        // report targets in view
        vision.reportTracking(telemetry);

        if (gamepad1.y) {
            // set the pose
            Motion.setPoseInches(Vision.inchX, Vision.inchY, Vision.degTheta);
        }

        // do some driving
        double forward = -0.7 * gamepad1.left_stick_y;
        double turn = 0.4 * gamepad1.right_stick_x;

        switch (state) {
            case STATE_START:
                if (Motion.finished() && gamepad1.a){
                    Motion.headTowardInches(60, -12);
                    state = State.STATE_TURN;
                }
                break;

            case STATE_TURN:
                if (Motion.finished() && gamepad1.a) {
                    Motion.moveInches(Motion.distanceToInches(60,-12));
                    state= State.STATE_MOVE;
                }
                break;

            case STATE_MOVE:
                if (Motion.finished() && gamepad1.a){
                    Motion.moveInches(-Motion.distanceToInches(12,-12));
                    state= State.STATE_BACK;
                }

        }





        // Motion.setPower(forward+turn, forward-turn);
    }

    @Override
    public void stop() {
        // turn off tracking
        vision.targets.deactivate();
    }

    /**
     * Set the robot pose based on alliance and starting position
     */
    public void setPose() {
        double robotBackDistance = 7.75;
        double dx = 36.0;
        double fy = 72.0 - robotBackDistance;

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