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

    Arm.Level level = Arm.Level.LEVEL1;
    Arm arm;
    ArmMotor armMotor;
    private enum State {
        STATE_INITIAL,
        STATE_FORWARD,
        STATE_TURN,
        STATE_DROP_FREIGHT,
        STATE_BACKUP,
        STATE_TURN_TOWARD_WAREHOUSE,
        STATE_FORWARD_WAREHOUSE1,
        STATE_FORWARD_WAREHOUSE2,

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

        Motion.init(hardwareMap);
        Motion.setPower(0.5);
        arm = new Arm();
        arm.init(hardwareMap);
        currState = AutoDrive.State.STATE_INITIAL;

        armMotor=new ArmMotor();
        armMotor.init(hardwareMap);



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

        // figure out the level
        switch (GameConfig.barCode) {
            case LEFT:
                level = Arm.Level.LEVEL1;
                break;
            case MIDDLE:
                level = Arm.Level.LEVEL2;
                break;
            default:
            case RIGHT:
                level = Arm.Level.LEVEL3;
                break;
        }
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
                Motion.moveInches(10.);
                arm.setLevel(level);
                //Motion.moveInches(20.); //used for testing
                newState(AutoDrive.State.STATE_FORWARD);
                telemetry.addData("Finished Initial State","");
                break;

            case STATE_FORWARD:
                // Loop while the motor is moving to the target
                if (Motion.finished()) {
                    // the robot has finished moving.
                    // give it new command
                    //Motion.headTowardInches(-12,-24);
                    Motion.headTowardInches(-24,-36); //USE THIS!!!
                    //Motion.headTowardInches()
                    //Motion.turnDegrees(-90); test the accuracy
                    //Motion.headTowardInches(0,0);

                    newState(State.STATE_TURN);
                    telemetry.addData("Finished Forward State","");
                }
                break;

            case STATE_TURN:
                // I'm executing a turn right now.

                // have we completed the turn?
                if (Motion.finished()) {
                    // we are headed toward the hub
                    // distance to center of hub
                    double distanceToMove = Motion.distanceToInches(-24,-36)
                            // distance to the center of the hub
                           // double dis = Motion.distanceToInches(-24,-36)
                            // need to make two corrections
                            // subtract the radius of the level
                            - level.radius
                            // subtract the extension of the arm
                            - arm.extension(level);


                    // calculate the distance we need to go
                    //arm.setLevel(level);
                    //double dis = Motion.distanceToInches(-24,-45);
                    //double dis = Motion.distanceToInches(0,0); //testing
                    // start the next movement
                    //Motion.moveInches(dis);

                    newState(State.STATE_DROP_FREIGHT);
                    //newState(State.STATE_STOP);

                }
                break;

            case STATE_DROP_FREIGHT:
                // have we finished moving?
                if (Motion.finished()) {
                    // we are at the alliance hub
                    armMotor.outtake();
                    Motion.moveInches(-5.0);
                    //newState(State.STATE_STOP);
                    newState(State.STATE_BACKUP);
                }
                break;

            case STATE_BACKUP:
                // have we finished backing up?
                if (Motion.finished()) {
                    // head toward the warehouse
                    Motion.headTowardInches(48,-48);
                    //newState(State.STATE_STOP);
                    newState(State.STATE_TURN_TOWARD_WAREHOUSE);

                }
                break;

            case STATE_TURN_TOWARD_WAREHOUSE:
                // have we finished the turn?
                if (Motion.finished()) {
                    double distanceMoveW = Motion.distanceToInches(10,-48);
                    // start moving that distance to the warehouse
                    Motion.moveInches(distanceMoveW);
                    //newState(State.STATE_STOP);
                    newState(State.STATE_FORWARD_WAREHOUSE1);
                }
                break;

            case STATE_FORWARD_WAREHOUSE1:
                if (Motion.finished()) {
                    double distanceMoveW = Motion.distanceToInches(60,-60);
                    Motion.setPower(0.8);
                    Motion.moveInches(distanceMoveW);
                    newState(State.STATE_FORWARD_WAREHOUSE2);

                }
                break;
            case STATE_FORWARD_WAREHOUSE2:
                if (Motion.finished()) {
                    newState(State.STATE_STOP);
                }
                break;

            case STATE_STOP:
                break;

        }
    }

    @Override
    public void stop() {
        Motion.setPower(0);
    }

    public void newState(AutoDrive.State newState) {
        //reset state time, change to next state
        currStateTime.reset();
        currState = newState;
    }

}


