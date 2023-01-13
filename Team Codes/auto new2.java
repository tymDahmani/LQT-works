
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


@Autonomous

public class Auto_New2 extends LinearOpMode {
    //// distance vars
//Ditance to the right
DistanceSensor distance1;
//distance values
double space1;
double space2;

//Movement Motors vars
DcMotor motor1;
DcMotor motor2;
DcMotor motor3;
DcMotor motor4;


//Arm Motor. corehex
DcMotorEx ArmMotor;

//slide. they move together
DcMotor slide1;
DcMotor slide2;

//servos. the hand
Servo servo1;


  ///the position of the robot in the field
int position = 0;
//if its 1 that means it will park to the right
//if its 2 that means it will park to the left


 //colorsensor
ColorSensor clr;


//colorPark
int colorpark = 0;


    @Override
    public void runOpMode() {
    //distance relates
    distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
    space1 = distance1.getDistance(DistanceUnit.CM);
    
    //Moving Motors relates
    motor1 = hardwareMap.get(DcMotor.class,"motorfrontleft");
    motor2 = hardwareMap.get(DcMotor.class,"motorfrontleft" );
    motor3 = hardwareMap.get(DcMotor.class,"motorbackright");
    motor4 = hardwareMap.get(DcMotor.class,"motorbackleft" );
    motor1.setDirection(DcMotorSimple.Direction.REVERSE);
    motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    
    //Modes for the moving controller
    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //Mode for the arm motor
    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //Mode for the slide motors and config
    slide1 = hardwareMap.get(DcMotor.class, "slide1");
    slide1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    slide2 = hardwareMap.get(DcMotor.class, "slide2");
    slide2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    
    //configuring the servos. the hand
    servo1 = hardwareMap.get(Servo.class, "servo1");
    
    
    //colorsensor config
    clr = hardwareMap.colorSensor.get("clr");
    

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            Check_distance();
            drive(00, 00);
            if(position == 1) {
                
            }
            
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
    
    void Check_distance(){
    // note that the distance sensor is on the right of the robot
    space1 = distance1.getDistance(DistanceUnit.METER);
  
        //comparing the values of the distance //right is higher
       if(space1 < 1.2192 /*one tile size*/){
            //means the robot is on the right side of the field
            position = 1;
          
        }
        //comparing the balues of the distance // left is higher
       if(space1 > 1.2192){
            //means the robot is on the left side of the field
            position = 2;
        }
    }
    
    
    /// the code for moving to the cone 
    void drive(int tar, int spd) {
        //setting the target position to the motors
        motorfrontright.setTargetPosition(tar);
        motorfrontleft.setTargetPosition(tar);
        motorbackright.setTargetPosition(tar);
        motorbackleft.setTargetPosition(tar);
        //telling the movement motor to go to the position they were told to
        motorfrontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfrontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbackleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbackright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //giving the movement motors the power to move
        motorfrontright.setPower(spd);
        motorfrontleft.setPower(spd);
        motorbackleft.setPower(spd);
        motorbackright.setPower(spd);
        
    }
    
    // read the cone func (color)
    void colorRead() {
        int currentColor;
        clr.enableLed(false);
        sleep(100);
        clr.enableLed(true);
        currentColor = Color.rgb(clr.red(), clr.green(), clr.blue());
        if (JavaUtil.colorToSaturation(currentColor) >= 0.6 && JavaUtil.colorToHue(currentColor) > 200 && JavaUtil.colorToHue(currentColor) < 235) {
          colorpark = 1;
          telemetry.update();
          clr.enableLed(false);
          
        } else {
          motor = true;
          telemetry.update();
        }
        telemetry.addData("saturation",JavaUtil.colorToSaturation(currentColor));
        telemetry.addData("monako",JavaUtil.colorToHue(currentColor));
        
    }
    
    
}
