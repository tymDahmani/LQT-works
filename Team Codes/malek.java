package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

Servo gripperServo;
DcMotor rightSlide;

@Autonomous

public class Auto_New2 extends LinearOpMode {
  
  gripperServo = hardwareMap.get(Servo.class, "gripperServo");
  rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
  
  gripperServo.setDirection(ServoSimple.Direction.REVERSE);
  rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  
  
  @Override
    public void runOpMode() {
      
       telemetry.addData("Status", "Initialized");
       telemetry.update();
       waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
          dropConePos();
          gripperDropCone();
          sleep(3000);
          coreGripper(700, 0.1);
          catchCone();
          dropConePos();
          sleep(3000);
          gripperDropCone();
          
          telemetry.addData("Status", "Running");
          telemetry.update();
        }
      
    }
  
  
}

public void dropConePos{
  rightSlide.setTargetPosition(90);
  rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  rightSlide.setPower(0.1);
  
}

public void gripperDropCone{
  gripperServo.setPower(1);
  sleep(200);
  gripperServo.setPower(0);
  
}

public void coreGripper(int ticks, int power) {
  rightSlide.setTargetPosition(ticks);
  rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  rightSlide.setPower(power);
  sleep(2000);
  
}

public void catchCone {
  gripperServo.setPower(1);
  sleep(2000);
  gripperServo.setPower(0);
  
}









  
  
  
  
  
  
  
  
