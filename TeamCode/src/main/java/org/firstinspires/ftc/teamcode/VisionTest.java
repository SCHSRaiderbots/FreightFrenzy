package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

@TeleOp(name="Vuforia Field Nav", group ="CodeDev")
public class VisionTest extends OpMode {

    // the Vision object
    Vision vision;

    /** the LynxModule serial number */
    String strSerialNumber;

    @Override
    public void init() {
        // get the serial number
        // TODO: use serial number to identify robot?
        strSerialNumber = LogDevice.getSerialNumber(hardwareMap);

        // report the LynxModules
        LogDevice.dumpFirmware(hardwareMap);

        // create the vision object
        vision = new Vision();

        // init the Vuforia localization engine
        vision.initVuforia(hardwareMap);

        // init tracking
        // TODO: does not play well with others...
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
        Motion.init(hardwareMap);
    }

    @Override
    public void init_loop() {
        // report the serial number during init
        // this causes an update, so it will flash the display
        // telemetry.addData("Serial Number", strSerialNumber);

        // report detected objects
        vision.reportDetections(telemetry);

        Motion.updateRobotPose();
    }

    @Override
    public void start() {
        // report current status
        telemetry.addData("Navigation", "On");
        vision.tfod.deactivate();

        if (vision.targets == null) {
            Log.d("vision.targets", "is null!");
        } else {
            vision.targets.activate();
        }

        // run using power
        Motion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        double forw = -gamepad1.left_stick_y;
        double turn = 0.7 * gamepad1.right_stick_x;

        Motion.setPower(forw+turn, forw-turn);
    }

    @Override
    public void stop() {
        // turn off tracking
        vision.targets.deactivate();
    }

}