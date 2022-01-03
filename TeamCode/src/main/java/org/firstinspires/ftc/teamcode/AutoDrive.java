package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDrive", group="CodeDev")
public class AutoDrive extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    static private DcMotorEx dcmotorLeft;
    static private DcMotorEx dcmotorRight;


    int test_num = 2;
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
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        dcmotorLeft  = hardwareMap.get(DcMotorEx.class, "left_drive");
        dcmotorRight = hardwareMap.get(DcMotorEx.class, "right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        dcmotorLeft.setDirection(DcMotor.Direction.FORWARD);
        dcmotorRight.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        //telemetry.addData("Status", "Initialized");
        dcmotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dcmotorLeft.setTargetPosition(0);
        dcmotorRight.setTargetPosition(0);

        dcmotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dcmotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //leftMotor.setPower(1);
        //rightMotor.setPower(1);

        currState = AutoDrive.State.STATE_INITIAL;
    }
    public void init_loop() {
    }
    public void start() {
        newState(State.STATE_INITIAL);
        runtime.reset();
    }
    public void loop() {
        telemetry.addData("current state", currState);
        switch(currState){
            // vision needs to occur in STATE_INTIAL
            case STATE_INITIAL:
                if (test_num == 2) {
                    newState(AutoDrive.State.STATE_FORWARD);
                    telemetry.addData("Object reached State", "");
                    dcmotorLeft.setTargetPosition(0);
                    dcmotorRight.setTargetPosition(0);
                }
                break;
            case STATE_FORWARD:
                // get current position,
                dcmotorLeft.setPower(.5);
                dcmotorRight.setPower(.5);
                Motion.moveForward(10);
                // Loop while the motor is moving to the target
                while(dcmotorLeft.isBusy()) {
                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting for the motor to reach its target");
                    telemetry.update();
                }
                newState(State.STATE_TURN);
            case STATE_DROP_FREIGHT:
                dcmotorLeft.setPower(.5);
                dcmotorRight.setPower(.5);
                int runtime = 0;
                //enter code for moving arm
                while(runtime >= 2){
                    continue;
                }
                newState(State.STATE_TURN);
            case STATE_TURN:
                dcmotorLeft.setPower(.5);
                dcmotorRight.setPower(.5);
                Motion.updateRobotPose();
                Motion.headTowardInches(-12,-24);
                double dis = Motion.distanceToInches(-12,-24);
                Motion.moveForward(dis);
                while(dcmotorLeft.isBusy()) {
                    // Let the drive team see that we're waiting on the motor
                    telemetry.addData("Status", "Waiting for the motor to reach its target");
                    telemetry.update();
                }
                newState(State.STATE_BACKUP);
            case STATE_BACKUP:









        }
        }


    public void stop() {
        //dcmotorLeft.setPower(0);
        //dcmotorRight.setPower(0);
    }



    public void newState(AutoDrive.State newState) {
        //reset state time, change to next state
        currStateTime.reset();
        currState = newState;
    }

}


