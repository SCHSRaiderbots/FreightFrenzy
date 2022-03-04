/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class Carousel{

    // TODO: gear ratio was changed; drive wheel diameter was changed

    DcMotorEx carouselMotor;
    double power = 1.0;


    public void init(HardwareMap hardwareMap) {
        // get the motor
        carouselMotor  = hardwareMap.get(DcMotorEx.class, "Carousel");

        // set the direction
        carouselMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carouselMotor.setVelocity(0);

        double rpm = 6000.0;
        double rps = rpm / 60;
        double ticksPerRev = 28;
        double f = 32000.0 / (ticksPerRev * rps);
        PIDFCoefficients pidfRUE = new PIDFCoefficients(10, 1, 0.0, f, MotorControlAlgorithm.PIDF);
        PIDFCoefficients pidfR2P = new PIDFCoefficients( 10, 0.0, 0.0, 0.0, MotorControlAlgorithm.PIDF);
        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfRUE);
        carouselMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidfR2P);

        // set motor mode
        carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set the power level
        carouselMotor.setPower(power);

        // log the motor settings
        LogDevice.dump("Carousel", carouselMotor);

        // set spin to zero
        spin(0.0);
    }

    /**
     * spin the carousel at a relative velocity
     * @param velocity a number between 0 and 1
     */
    public void spin(double velocity) {
        // max spin
        // using 1.0, a perimeter setting spins off
        //   set duck tail at the black cap and step on the gas, duck falls off
        // using 0.8, a perimeter setting spins off
        //   set at danger, and it will spin out.
        //   set duck tail at the black cap and works unless the duck rolls over
        // using 0.7, a perimeter setting will spin off
        //   a setting on inch in will not spin off
        //   the duck jumps during the initial acceleration, so a soft start would be better
        // using 0.6, a perimeter setting will spin off
        //   a setting one inch in will not spin off
        // using 0.5, a perimeter setting will spin off before hitting the sweeper plate
        //   a mid setting will not spin off
        // using 0.4, it will not spin off if set on the periphery
        double rpsMax = 0.8;

        // convert max spin to ticks
        double ticksMax = rpsMax
                * 28.0 * Motion.HD_HEX_GEAR_CART_5_1 * Motion.HD_HEX_GEAR_CART_4_1
                * (15.0 * 25.4 / 90.0);

        // spin the carousel at a scaled velocity
        carouselMotor.setVelocity(velocity * ticksMax);
    }

    /**
     * Get the velocity in Revolutions per second.
     * @return revolutions per second.
     */
    public double getRelativeVelocity() {
        // this number should be absolute ticks per second
        double ticksPerSecond = carouselMotor.getVelocity();
        // we have an ultra planetary with 4:1 and 5:1 cartridges
        double rps = ticksPerSecond / (28.0 * Motion.HD_HEX_GEAR_CART_4_1 * Motion.HD_HEX_GEAR_CART_5_1);

        // the 15-inch carousel will spin slower than the motor rpm.
        // drive wheel is 90 mm
        return rps * (90.0 / (15.0 * 25.4));
    }

}
