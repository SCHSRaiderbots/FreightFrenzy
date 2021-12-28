package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "Carousol Wheel", group = "Linear Mode")
public class movingMotor extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx carousolWheel;
    @Override
    // everything that needs to be iniialized before starting the code
    public void runOpMode(){
      telemetry.addData("Status", "Initalized");
      telemetry.update();

      carousolWheel=hardwareMap.get(DcMotorEx.class, "motorCarousel");
      carousolWheel.setDirection(DcMotorEx.Direction.FORWARD);

      waitForStart();
      runtime.reset();

      while (opModeIsActive()){
          double motorPower;
          if(gamepad1.a) {
              carousolWheel.setVelocity(-2500);
          }
          if(gamepad1.b) {
              carousolWheel.setVelocity(0.0);
          }
          telemetry.addData("status", "MotorPower: " + carousolWheel.getVelocity());
          telemetry.update();
      }
    }

}
