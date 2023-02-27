package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="quantaMainCode_#1", group="Linear Opmode")

public class quantaMainCode_#1 extends LinearOpMode {
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor Slide; // (the arm motor)
    // (the slide motors)
    DcMotor MotorRight;
    DcMotor MotorLeft;
    // Servo
    Servo GripperServo





@Override
public void runOpMode() {

    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    DcMotor Slide;
    DcMotor motorRight;
    DcMotor motorLeft;





    motor1=   hardwareMap.get(DcMotor.class, "Drivetrain RB");
    motor2= hardwareMap.get(DcMotor.class, "Drivetrain LB");
    motor3= hardwareMap.get(DcMotor.class, "Drivetrain RF");
    motor4= hardwareMap.get(DcMotor.class, "Drivetrain LF");


    motor4.setDirection(DcMotor.Direction.REVERSE);

    Slide = hardwareMap.get(DcMotor.class,"Right Slide");
    Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    motorRight = hardwareMap.get(DcMotor.class,"Motor Right");
    motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    motorLeft = hardwareMap.get(DcMotor.class,"Motor Left");
    motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    motor4.setMode(DcMotor.RunMode.RUN_USING_ENCODER);










        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            initCone();
            move(00, 00, 00, 00 /* those are the target position ticks */, 00, 00, 00, 00 /* those are the power the motors will use */);
            grab_drop_cone();



            telemetry.update();
        }



    void move(int pos1, int pos2, int pos3, int pos4, int pwr1, int pwr2, int pwr3, int pwr4) {
        motor4.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor4.setTargetPosition(pos1);
        motor2.setTargetPosition(pos2);
        motor3.setTargetPosition(pos3);
        motor1.setTargetPosition(pos4);
        motor4.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor4.setPower(pwr1);
        motor2.setPower(pwr2);
        motor3.setPower(pwr3);
        motor1.setPower(pwr4);




    }

    // function to put first cone that the robot's already holding
    void initCone() {
        void Drop_Cone_Position_arm() {
          Slide.setTargetPosition(200);
          Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          Slide.setPower(0.1);
        }

        /**
         * Describe this function...
         */
        void Gripper_Drop_Cone() {
          // open servo for 200ms to drop the cone
          Servo.setPower(1);
          sleep(400);
          // close servo so the gripper can fit in the Arm to head back to the intake position
          Servo.setPower(0);
        }


        void Slides_Home_Position(int posL, int posR, int power) {
          MotorRight.setTargetPosition(posR);
          MotorLeft.setTargetPosition(posL);
          MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          MotorRight.setPower(power);
          MotorLeft.setPower(power);
        }


    }






    // fuction for all the grab drop cone functions
    void grab_drop_cone() {
        Slides_Motion(00, 00, 00, 00);
        Arm(00, 00);
        Catch_Cone(); // edit values if needed
        Home_Position(); // edit values if needed
        Drop_Cone_Position_arm(); // edit values if needed
        Gripper_Drop_Cone(); // edit values if needed
        Slides_Home_Position(00, 00, 00);


    }





    // grab-drop_cone functions:

    private void Slides_Motion(int RightTicks, int LeftTicks, int RightPower, int LeftPower) {
      MotorRight.setTargetPosition(RightTicks);
      MotorLeft.setTargetPosition(LeftTicks);
      MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      MotorRight.setPower(RightPower);
      MotorLeft.setPower(LeftPower);
    }

    /**
     * Describe this function...
     */
    private void Arm(int Ticks, int Power) {
      Slide.setTargetPosition(Ticks);
      Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Slide.setPower(Power);
    }


    // we'll amke it with ticks
    private void Catch_Cone() {
      // open servo to catch the cone
      Servo.setPower(1);
      // keep open to catch the cone
      sleep(600);
      // Close gripper to catch the cone
      Servo.setPower(0);
    }


    /**
     * Describe this function...
     */
    private void Home_Position() {
      Slide.setTargetPosition(20);
      Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Slide.setPower(0.1);
    }

    /**
     * Describe this function...
     */
    private void Drop_Cone_Position_arm() {
      Slide.setTargetPosition(200);
      Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      Slide.setPower(0.1);
    }

    /**
     * Describe this function...
     */
    private void Gripper_Drop_Cone() {
      // open servo for 200ms to drop the cone
      Servo.setPower(1);
      sleep(400);
      // close servo so the gripper can fit in the Arm to head back to the intake position
      Servo.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void Slides_Home_Position(int posL, int posR, int power) {
      MotorRight.setTargetPosition(posR);
      MotorLeft.setTargetPosition(posL);
      MotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      MotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      MotorRight.setPower(power);
      MotorLeft.setPower(power);
    }
  }





}
