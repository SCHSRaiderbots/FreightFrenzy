package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
//check the config for the AutoDrive code to make sure that left_Drive is assigned to right motor
// and right_drive is assigned to left motor
@Autonomous(name="AutoDrive", group="CodeDev")
public class AutoDrive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static private DcMotorEx dcmotorLeft;
    static private DcMotorEx dcmotorRight;

    private enum State {
        STATE_INITIAL,
        STATE_FORWARD,
        STATE_TURN,
        STATE_DROP_FREIGHT,
        STATE_BACKUP,
        STATE_TURN_TOWARD_WAREHOUSE,
        STATE_FORWARD_WAREHOUSE,

        STATE_TEST_1,
        STATE_TEST_2,

        STATE_STOP
    }
    private AutoDrive.State currState;
    private ElapsedTime currStateTime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dcmotorLeft  = hardwareMap.get(DcMotorEx.class, "left_drive");
        dcmotorRight = hardwareMap.get(DcMotorEx.class, "right_drive");
        Motion.setRobotMotors(dcmotorLeft, dcmotorRight);
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dcmotorRight.setDirection(DcMotor.Direction.REVERSE);
        dcmotorLeft.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
        dcmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotorLeft.setTargetPosition(0);
        dcmotorRight.setTargetPosition(0);

        dcmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set the powers
        dcmotorLeft.setPower(0.4);
        dcmotorRight.setPower(0.4);

        currState = AutoDrive.State.STATE_INITIAL;
        Motion.setRobotDims2020();

    }

    @Override
    public void init_loop() {
        Motion.updateRobotPose();

        // detect where the duck is
    }

    /**
     * At start() we should know the position of the duck.
     */
    @Override
    public void start() {
        newState(State.STATE_INITIAL);
        runtime.reset();
        //Motion.setPoseInches(-36.0,-63.0, 90); original points
        Motion.setPoseInches(-36.0,-61.5, 90);

    }

    @Override
    public void loop() {
        Motion.updateRobotPose();
        telemetry.addData("Position", "%.02f %.02f %.02f", Motion.xPoseInches, Motion.yPoseInches, Motion.thetaPoseDegrees);
        // report the current state
        telemetry.addData("current state", currState);

        switch(currState){
            // we need to start moving
            case STATE_INITIAL:
                // start moving
                //Motion.moveInches(10.);
                Motion.moveInches(20.); //used for testing
                newState(AutoDrive.State.STATE_FORWARD);
                telemetry.addData("Finished Initial State","");
                break;

            case STATE_FORWARD:
                // Loop while the motor is moving to the target
                if (!dcmotorLeft.isBusy() && !dcmotorRight.isBusy()) {

                    // the robot has finished moving.
                    // give it new command
                    //Motion.headTowardInches(-12,-24);
                    //Motion.headTowardInches(-24,-36); USE THIS!!!
                    //Motion.headTowardInches()
                    //Motion.turnDegrees(-90); test the accuracy
                    Motion.headTowardInches(0,0);

                    newState(State.STATE_TURN);
                    telemetry.addData("Finished Forward State","");
                }
                break;

            case STATE_TURN:
                // I'm executing a turn right now.

                // have we completed the turn?
                if (!dcmotorLeft.isBusy() && !dcmotorRight.isBusy()) {
                    // calculate the distance we need to go

                   //double dis = Motion.distanceToInches(-24,-45);
                    double dis = Motion.distanceToInches(0,0); //testing
                    // start the next movement
                    Motion.moveInches(dis);

                    newState(State.STATE_DROP_FREIGHT);
                    newState(State.STATE_STOP);

                }
                break;

            case STATE_DROP_FREIGHT:
                // have we finished moving?
                if (!dcmotorLeft.isBusy() && !dcmotorRight.isBusy()) {
                    // we are at the alliance hub
                    Motion.moveInches(-5.0);
                    newState(State.STATE_STOP);
                    // newState(State.STATE_BACKUP);
                }
                break;

            case STATE_BACKUP:
                // have we finished backing up?
                if (!dcmotorLeft.isBusy() && !dcmotorRight.isBusy()) {
                    // head toward the warehouse
                    Motion.headTowardInches(48,-48);
                    newState(State.STATE_TURN_TOWARD_WAREHOUSE);
                    //newState(State.STATE_STOP);
                }
                break;

            case STATE_TURN_TOWARD_WAREHOUSE:
                // have we finished the turn?
                if (!dcmotorLeft.isBusy() && !dcmotorRight.isBusy()) {
                    double distanceMoveW = Motion.distanceToInches(48,-48);
                    // start moving that distance to the warehouse
                    Motion.moveInches(distanceMoveW);
                    newState(State.STATE_FORWARD_WAREHOUSE);
                }
                break;

            case STATE_FORWARD_WAREHOUSE:
                if (!dcmotorLeft.isBusy() && !dcmotorRight.isBusy()) {

                    newState(State.STATE_STOP);
                }
                break;

            case STATE_STOP:
                break;

        }
    }

    @Override
    public void stop() {
        dcmotorLeft.setPower(0);
        dcmotorRight.setPower(0);
    }

    public void newState(AutoDrive.State newState) {
        //reset state time, change to next state
        currStateTime.reset();
        currState = newState;
    }

}


