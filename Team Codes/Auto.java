package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Basic: Omni Linear OpMode", group="Linear Opmode")


public class Auto extends LinearOpMode {
   //// distance vars
//Ditance to the right
DistanceSensor distance1;
//distance to the left
DistanceSensor distance2;
//distance values
double space1;
double space2;

//Movement Motors vars
DcMotor motorfrontright;
DcMotor motorfrontleft;
DcMotor motorbackright;
DcMotor motorbackleft;


//Arm Motor
DcMotorEx ArmMotor;

//servos
Servo servo1;
Servo servo2;

//color var
ColorSensor colorsens;

  ///parking postion
int parking = 0;
//if its 1 that means it will park to the right
//if its 2 that means it will park to the left


 //colorsensords
ColorSensor color;
double Red;
double Blue;
double Green;

    @Override
    public void runOpMode() {
    //distance relates
    distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
    distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
    space1 = distance1.getDistance(DistanceUnit.CM);
    space2 = distance2.getDistance(DistanceUnit.CM);
    
    //Moving Motors relates
    motorfrontright =hardwareMap.get(DcMotor.class,"motorfrontleft");
    motorfrontleft = hardwareMap.get(DcMotor.class,"motorfrontleft" );
    motorbackright = hardwareMap.get(DcMotor.class,"motorbackright");
    motorbackleft = hardwareMap.get(DcMotor.class,"motorbackleft" );
    motorfrontright.setDirection(DcMotorSimple.Direction.REVERSE);
    motorbackright.setDirection(DcMotorSimple.Direction.REVERSE);
    
    //Modes for the moving controller
    motorfrontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorfrontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorfrontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorfrontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorbackright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorbackright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motorbackleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorbackleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //Mode for the arm motor
    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    
    //configuing the servos
    servo1 = hardwareMap.get(Servo.class, "servo1");
    servo2 = hardwareMap.get(Servo.class, "servo2");
    
    
    
    
    
    //colorsensor config
    colorsens = hardwareMap.colorSensor.get("clr");
    
   

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        

        waitForStart();
      


        while (opModeIsActive()) {
            Move_Forward(00,00);
            while(motorfrontright.isBusy() ||motorfrontleft.isBusy()||motorbackleft.isBusy()||motorbackright.isBusy()){
            }
            Color_read();
            Check_Distance();
            Move_Forward(00,00);
            Color_read();
            Turn_After_Clr();
            forward_after_clr(00,00);
            left_to_Cone(00,00);
            telemetry.update();
        }
        
        
    }
    
    void Check_Distance()
    {
       // getting the values for the distance sensors
       space1 = distance1.getDistance(DistanceUnit.CM);
       space2 = distance2.getDistance(DistanceUnit.CM);
       
       //comparing the values of the distance //right is higher
       if(space1>space2){
           //means the robot is going to park to the right corner
           parking = 1;
           
       }
       //comparing the balues of the distance // left is higher
       if(space1<space2){
           //means the robot is parking to the left corner
           parking =2;
       }
      
       
    }
    
    
    //Note THE ROBOT WILL BE TURNING WITH THIS CODE
    void Move_Forward(int tar, int spd){
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
    
    
    void Color_read(){
        Red = colorsens.red();
        Blue = colorsens.blue();
        Green = colorsens.green();
    }
    
    
    void Turn_After_Clr(){
        motorfrontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfrontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
         //setting the target position to the motors
        motorfrontright.setTargetPosition(00);
        motorfrontleft.setTargetPosition(00);
        
        //telling the movement motor to go to the position they were told to
        motorfrontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfrontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        //giving the movement motors the power to move
        motorfrontright.setPower(00);
        motorfrontleft.setPower(00);
    }
    
    void forward_after_clr(int tar, int spd){
        //setting the target position to the motors
        motorfrontright.setTargetPosition(tar);
        motorfrontleft.setTargetPosition(tar);
        motorbackright.setTargetPosition(tar);
        motorbackleft.setTargetPosition(tar);
        //telling the movement motor to go to the position they were told to
        motorfrontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfrontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbackleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbackright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //giving the movement motors the power to move
        motorfrontright.setPower(spd);
        motorfrontleft.setPower(spd);
        motorbackleft.setPower(spd);
        motorbackright.setPower(spd);
    }
    
    
    void left_to_Cone(int tar, int spd){
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
    
    void Desposer(int tar,int spd, int Pos){
        ArmMotor.setTargetPosition(tar);
        ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmMotor.setVelocity(spd);
        while(ArmMotor.isBusy()){
            
        }
        servo1.setPosition(Pos);
        servo2.setPosition(Pos);
    }
}
