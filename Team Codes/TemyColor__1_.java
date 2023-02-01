/*
Copyright 2023 FIRST Tech Challenge Team 19351

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class TemyColor__1_ extends LinearOpMode {
    private Blinker control_Hub;
    private CRServo gripper_Servo;
    private DcMotor motor_Left;
    private DcMotor motor_Right;
    private DcMotor right_Slide;
    private DcMotorEx LF;
    private DcMotorEx LB;
    private DcMotorEx RF;
    private DcMotorEx RB;
    private Gyroscope imu;
    ColorSensor clr;

    @Override
    public void runOpMode() {
        control_Hub = hardwareMap.get(Blinker.class, "Control Hub");
        motor_Left = hardwareMap.get(DcMotor.class, "Motor Left");
        motor_Right = hardwareMap.get(DcMotor.class, "Motor Right");
        // arm motor
        right_Slide = hardwareMap.get(DcMotor.class, "Right Slide");
        // drive train motors
        LF = hardwareMap.get(DcMotorEx.class, "Drivetrain LF");
        LB = hardwareMap.get(DcMotorEx.class, "Drivetrain LB");
        RF = hardwareMap.get(DcMotorEx.class, "Drivetrain RF");
        RB = hardwareMap.get(DcMotorEx.class, "Drivetrain RB");
        imu = hardwareMap.get(Gyroscope.class, "imu");
        clr = hardwareMap.get(ColorSensor.class, "clr");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            testRGB();
            
            for (int count = 0; count <1; count++)
                if(clr.red() > 120 && clr.red() < 370) {
                    moveN(30, 0.4);
                    telemetry.addData("robot moves? ", "yes");
                    telemetry.update();
                } else if(clr.blue() >= 150 && clr.blue() <= 700) {
                    
                    telemetry.addData("robot moves?", "no");
                    telemetry.update();
                }
            
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        
        
    }
    public void move_backward(int tar, double spd) {
            // stop and reset mode, run using encoder mode
            LF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            LB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            LB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            RF.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RF.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            RB.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            RB.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // run to target pos mode
            LF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            LB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RF.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RB.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            // target position given
            LF.setTargetPosition(tar);
            LB.setTargetPosition(tar);
            RF.setTargetPosition(-tar);
            RB.setTargetPosition(-tar);
            // the motor speed
            LF.setPower(spd);
            LB.setPower(spd);
            RF.setPower(spd);
            RB.setPower(spd);
            
            
            
        }
        
    public void moveN(int tar, double spd) {
            // stop and reset mode, run using encoder mode
            right_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            right_Slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // run to target pos mode
            right_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            // target position given
            right_Slide.setTargetPosition(-tar);
            // the motor speed
            right_Slide.setPower(spd);
            
    }
    
    void testRGB() {
             
            // show colors RGB
            telemetry.addData("red", clr.red());
            telemetry.addData("green", clr.green());
            telemetry.addData("blue", clr.blue());
            
            
        
    }
    
    void testCode(int tar, double spd) {
        
        // stop and reset mode, run using encoder mode
            right_Slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            right_Slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            // run to target pos mode
            right_Slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            // target position given
            right_Slide.setTargetPosition(tar);
            // the motor speed
            right_Slide.setPower(spd);
            
            
    }
    
    void testCodev2() {
        // if red >= 5, move slides -
            if (clr.red() >= 260 && clr.red() <= 300) {
                // func for the movement
                // move_backward(1000, 0.6);
                telemetry.addData("workss? ", "yup" );
                telemetry.update(); 
            } else if (clr.red() < 110) {
                // idle();
                telemetry.addData("works fine? ", "no" );
                telemetry.update();
            } else {
                telemetry.addData("works fine? ", "noooope");
            }
            
            
            // if blue >= 5, move slides +
            // if (clr.blue() >= 5) {
            //     // func for the movement   
            //     moveP(1000, 0.6);
            //     telemetry.addData("wroks?", "yes" );
            //     telemetry.update();
            // }
    }
    
}
