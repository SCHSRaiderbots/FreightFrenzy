package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XZY;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Class that keeps the vision code in one place.
 * The OpModes that want to use vision should call the methods in this class.
 * Steps are
 *   vision = new Vision();
 *   vision.initVuforia(hardwareMap);
 *   vision.initTfod(hardwareMap);
 *
 *   TODO: static class? static methods?
 */
public class Vision {
    /**
     * Put the Vuforia key in one place.
     *
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data.
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    static final String VUFORIA_KEY =
            "AUnX7nP/////AAABmZjfOTd2skx4p/r+LBA29VQAFar5mbPnEfGtcl78mMIqK+EtsUOR33zwyiDCmj1oYMUx0P4eWZGi6EMhZgTM66/5llx5azKwGGxGmTJUGotbAekyZgxYR7SWDme6xMYGR68jZcR9rkvJxfB1ZKFytPXWeRpwzSAQJ0VACF/hdguUyfA6SSkF2dnc/iH76TkSV3hA4zz0v3wjHfQmmNBvrtgPklvfOTX2f+G5tBfBq75PEx52LaX+tOPTtBajR9MFwVT26kcqFz2GJCEBgjO3PX1St0xNJBqbbudKvZ+B/6xWuVhwHVqwOgy/RsuHLBFskh4n9Ec1xnuB9uCnQXrrliEtcR1TbnmIEYTX6FZtxF5H";

    /** the webcam */
    WebcamName webcamName;

    /** the Vuforia localization engine. */
    VuforiaLocalizer vuforia;

    /** the TensorFlow Object Detection engine. */
    TFObjectDetector tfod;

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = 6 * mmPerInch;          // the height of the center of the target image above the floor
    private static final float halfField        = 72 * mmPerInch;
    private static final float halfTile         = 12 * mmPerInch;
    private static final float oneAndHalfTile   = 36 * mmPerInch;

    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    VuforiaTrackables targets   = null ;
    // the trackable navigation targets
    List<VuforiaTrackable> allTrackables = new ArrayList<>();


    /**
     * Initialize the Vuforia localization engine.
     */
    void initVuforia(HardwareMap hardwareMap) {
        // get the webcam name
        webcamName =  hardwareMap.get(WebcamName.class, "Webcam 1");

        // create vuforia parameter object
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera-preview is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        // int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // set the parameter values...
        // set the key
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        // set the webcam
        parameters.cameraName = webcamName;
        // Turn off Extended tracking.  Set this true if you want Vuforia to track beyond the target.
        // TODO: what does this mean?
        parameters.useExtendedTracking = false; //stops vision once the target objects are not in view

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.5f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;

        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS); //what and why is it needed
    }

    void reportDetections(Telemetry telemetry) {
        Locale locale = null;

        // do we have an object detector
        if (tfod == null) {
            telemetry.addData("TFOD", "no detector!");
        }
        else {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            // if we have new recognition information...
            if (updatedRecognitions != null) {
                // report the number of objects detected.
                telemetry.addData("# Object Detected", updatedRecognitions.size());

                // step through the list of recognitions and display boundary info.
                for (Recognition recognition : updatedRecognitions) {
                    int width = recognition.getImageWidth();
                    int height = recognition.getImageHeight();
                    telemetry.addData("image size", "width %d height %d",
                            width,height);
                    // TODO: filter the recognitions
                    // is it a reasonable object (e.g., just interested in ducks)
                    // is it a reasonable size (sometimes recognitions are huge)
                    // is it at a reasonable position (expected position)
                    // is it he best confidence
                    // at another time, we may want to identify objects to pick up
                    telemetry.addData(recognition.getLabel(),
                            String.format(locale, "(%.01f, %.01f) (%.01f, %.01f) %.03f",
                                    recognition.getLeft(), recognition.getTop(),
                                    recognition.getRight(), recognition.getBottom(),
                                    recognition.getConfidence()));

                    // this test ("==") relies on the strings being interned!
                    //   recognition.getLabel() == "Duck"
                    // this test will work as long as getLabel() does not return null
                    //   recognition.getLabel().equals("Duck")
                    if (recognition.getLabel().equals("Duck")) {
                        float center= (recognition.getLeft()+recognition.getRight())/2;
                        float delta=width/6.0f;
                        float d1=(width/2.0f)-delta;
                        float d2=(width/2.0f)+delta;
                        if (center<d1){
                            GameConfig.barCode=GameConfig.BarCode.LEFT;
                        }else if (center<d2){
                            GameConfig.barCode= GameConfig.BarCode.MIDDLE;
                        }else{
                            GameConfig.barCode= GameConfig.BarCode.RIGHT;
                        }
                    }

                    telemetry.addData("barCode:",GameConfig.barCode);
                } // getLeft() etc gets the pixel coordinates for an image in tensorflow
            }
        }

    }

    /**
     * Initialize tracking by loading the trackables.
     * Position information depends on the robot!
     * Does not activate tracking!
     */
    void initTracking() {
        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        targets = vuforia.loadTrackablesFromAsset("FreightFrenzy");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        // clear the list (we do not want a second invocation keeping the old targets.
        allTrackables = new ArrayList<>();
        allTrackables.addAll(targets);

        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Name and locate each trackable object
        identifyTarget(0, "Blue Storage", -halfField, oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(1, "Blue Alliance Wall", halfTile, halfField, mmTargetHeight, 90, 0, 0);
        identifyTarget(2, "Red Storage", -halfField, -oneAndHalfTile, mmTargetHeight, 90, 0, 90);
        identifyTarget(3, "Red Alliance Wall", halfTile, -halfField, mmTargetHeight, 90, 0, 180);

        /*
         * Create a transformation matrix describing where the camera is on the robot.
         *
         * Info:  The coordinate frame for the robot looks the same as the field.
         * The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
         * Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
         *
         * For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
         * with the wide (horizontal) axis of the camera aligned with the X axis, and
         * the Narrow (vertical) axis of the camera aligned with the Y axis
         *
         * But, this example assumes that the camera is actually facing forward out the front of the robot.
         * So, the "default" camera position requires two rotations to get it oriented correctly.
         * 1) First it must be rotated +90 degrees around the X axis to get it horizontal (its now facing out the right side of the robot)
         * 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
         *
         * Finally the camera can be translated to its actual mounting position on the robot.
         *      In this example, it is centered on the robot (left-to-right and front-to-back), and 6 inches above ground level.
         */
        // TODO: fix camera displacement issues
        final float CAMERA_FORWARD_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the forward distance from the center of the robot to the camera lens
        final float CAMERA_VERTICAL_DISPLACEMENT = 6.0f * mmPerInch;   // eg: Camera is 6 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0.0f * mmPerInch;   // eg: Enter the left distance from the center of the robot to the camera lens

        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XZY, DEGREES, 90, 90, 0));

        // Let all the trackable listeners know where the camera is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setCameraLocationOnRobot(webcamName, cameraLocationOnRobot);
        }

        // TODO: did not activate()
        // targets.activate();

    }

    /***
     * Identify a target by naming it, and setting its position and orientation on the field
     * @param targetIndex index of the target
     * @param targetName name to use for that index
     * @param dx, dy, dz  Target offsets in x,y,z axes
     * @param rx, ry, rz  Target rotations in x,y,z axes
     */
    void identifyTarget(int targetIndex, String targetName, float dx, float dy, float dz, float rx, float ry, float rz) {
        VuforiaTrackable aTarget = targets.get(targetIndex);
        aTarget.setName(targetName);
        aTarget.setLocation(OpenGLMatrix.translation(dx, dy, dz)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, rx, ry, rz)));
    }

    private OpenGLMatrix lastLocation = null;

    /**
     * Report a target in view.
     * Need to fix this logic to always report a tracking result.
     * Split off the actual tracking result.
     * Split off the targetVisible logic to indicate a new result.
     * @param telemetry telemetry object to report to screen
     */
    void reportTracking(Telemetry telemetry) {
        // assume nothing is visible
        boolean targetVisible = false;

        // check all the trackable targets to see which one (if any) is visible.
        for (VuforiaTrackable trackable : allTrackables) {
            if (((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                // found a visible target
                telemetry.addData("Visible Target", trackable.getName());

                // remember we saw it
                targetVisible = true;

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();
                // if the location is not null, then use it
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }
                break;
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible) {
            // express position (translation) of robot in inches.
            // TODO: lastLocation may be null!
            VectorF translation = lastLocation.getTranslation();

            // report the position
            telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            // report the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            // nothing was visible
            telemetry.addData("Visible Target", "none");
        }
    }
}
