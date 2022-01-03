///* Copyright (c) 2017 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
///**
// * This file contains an example of an iterative (Non-Linear) "OpMode".
// * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
// * The names of OpModes appear on the menu of the FTC Driver Station.
// * When an selection is made from the menu, the corresponding OpMode
// * class is instantiated on the Robot Controller and executed.
// *
// * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
// * It includes all the skeletal structure that all iterative OpModes contain.
// *
// * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
// * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
// */
//
//@Autonomous(name="Basic: Iterative OpMode", group="Iterative Opmode")
//public class DriveNew extends OpMode
//{
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor leftMotor = null;
//    private DcMotor rightMotor = null;
//
//
//    int test_num = 2;
//
//    private enum State {
//        STATE_INITIAL,
//        STATE_FORWARD,
//        STATE_TURN,
//        STATE_DROP_FREIGHT,
//        STATE_BACKUP,
//        STATE_TURN_TOWARD_WAREHOUSE,
//        STATE_FORWARD_WAREHOUSE,
//
//        STATE_TEST_1,
//        STATE_TEST_2,
//
//        STATE_STOP
//    }
//
//    private State currState;
//    private ElapsedTime currStateTime = new ElapsedTime();
//
//
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        telemetry.addData("Status", "Initialized");
//
//        // Initialize the hardware variables. Note that the strings used here as parameters
//        // to 'get' must correspond to the names assigned during the robot configuration
//        // step (using the FTC Robot Controller app on the phone).
//        leftMotor  = hardwareMap.get(DcMotor.class, "left_drive");
//        rightMotor = hardwareMap.get(DcMotor.class, "right_drive");
//
//        // Most robots need the motor on one side to be reversed to drive forward
//        // Reverse the motor that runs backwards when connected directly to the battery
//        leftMotor.setDirection(DcMotor.Direction.FORWARD);
//        rightMotor.setDirection(DcMotor.Direction.REVERSE);
//
//        // Tell the driver that initialization is complete.
//        //telemetry.addData("Status", "Initialized");
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setTargetPosition(0);
//        rightMotor.setTargetPosition(0);
//
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setPower(1);
//        rightMotor.setPower(1);
//
//        currState = State.STATE_INITIAL;
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        newState(State.STATE_INITIAL);
//        runtime.reset();
//    }
//
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//        // Setup a variable for each drive wheel to save power level for telemetry
//
//        telemetry.addData("current state", currState);
//
//        switch (currState) {
//
//
//            case STATE_INITIAL:
//                if (test_num == 2) {
//                    newState(State.STATE_FORWARD);
//                    telemetry.addData("Object reached State", "");
//                    leftMotor.setTargetPosition(0);
//                    rightMotor.setTargetPosition(0);
//
//                }
//
//
//                break;
//
//            case STATE_FORWARD:
//                telemetry.addData("Left Encoder Counts",leftMotor.getCurrentPosition());
//                telemetry.addData("Right Encoder Counts", rightMotor.getCurrentPosition());
//
//
//                leftMotor.setTargetPosition(400);
//                rightMotor.setTargetPosition(400);
//
//                if(leftMotor.getTargetPosition() >= 396 && rightMotor.getTargetPosition() >= 396){
//                    telemetry.addData("Finished Forward State","");
//                    newState(State.STATE_TURN);
//
//                    break;
//                }
//
//
//
//
//            case STATE_TURN:
//                rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                leftMotor.setPower(-0.6);
//                rightMotor.setPower(0.6);
//
//                leftMotor.setTargetPosition(100);
//                rightMotor.setTargetPosition(100);
//
//                if (leftMotor.getTargetPosition()>= 96 && rightMotor.getTargetPosition()>= 96){
//                    telemetry.addData("Finished Turn State","");
//                    newState(State.STATE_DROP_FREIGHT);
//
//                    break;
//
//
//                }
//
//            case STATE_DROP_FREIGHT:
//                if (test_num == 2) {
//                    newState(State.STATE_BACKUP);
//                    telemetry.addData("Finished Freight State", "");
//
//                    break;
//                }
//
//            case STATE_BACKUP:
//                rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
//                leftMotor.setPower(0.6);
//                rightMotor.setPower(-0.6);
//
//                leftMotor.setTargetPosition(100);
//                rightMotor.setTargetPosition(100);
//
//                if (leftMotor.getTargetPosition()>= 96 && rightMotor.getTargetPosition()>= 96){
//                    telemetry.addData("Finished Backup State","");
//                    newState(State.STATE_TURN_TOWARD_WAREHOUSE);
//
//                    break;
//
//
//                }
//
//            case STATE_TURN_TOWARD_WAREHOUSE://ask abt if coordinates itself can be used to move a robot to certain place
//                leftMotor.setPower(-0.6);
//                rightMotor.setPower(0.6);
//
//                leftMotor.setTargetPosition(100);
//                rightMotor.setTargetPosition(100);
//
//                if (leftMotor.getTargetPosition()>= 96 && rightMotor.getTargetPosition()>= 96){
//                    telemetry.addData("Finished Backup State","");
//                    newState(State.STATE_TURN_TOWARD_WAREHOUSE);
//
//                    break;
//
//
//                }
//
//            case STATE_FORWARD_WAREHOUSE:
//
//
//                leftMotor.setTargetPosition(400);
//                rightMotor.setTargetPosition(400);
//
//                if(leftMotor.getTargetPosition() >= 396 && rightMotor.getTargetPosition() >= 396){
//                    telemetry.addData("Finished Forward Warehouse State","");
//                    newState(State.STATE_STOP);
//
//                    break;
//                }
//
//            case STATE_STOP:
//                leftMotor.setPower(0);
//                rightMotor.setPower(0);
//
//
//
//
//
//
//
//
//
//
//
//        }
//
//
//
//
//
//
//
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//        leftMotor.setPower(0);
//        rightMotor.setPower(0);
//    }
//
//    public void newState(State newState) {
//        //reset state time, change to next state
//        currStateTime.reset();
//        currState = newState;
//    }
//
//    public void encodersAtZero(){
//        leftMotor.getCurrentPosition();
//        rightMotor.getCurrentPosition();
//    }
//
//}
//
//
//
//
//
