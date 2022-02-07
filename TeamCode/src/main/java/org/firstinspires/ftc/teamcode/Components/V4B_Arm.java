package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class V4B_Arm {
    Caching_Servo left_servo;
    Caching_Servo right_servo;
    public Caching_Servo release_servo;

    private boolean out = false;
    private boolean release = false;
    ElapsedTime time = new ElapsedTime();

    public Caching_Servo front;

    public static boolean partialToggle;
    boolean lowGoal = false;

    /*

         100 degree angle:
            Right:0.6
            Left: 0.385;

          150 degree angle:
          Right: 0.185
          Left:0.8

          Low Level:
          Right:0.1
          Left:0.89

          Low Level 2:
          Right:0.05
          Left:0.945
     */

    public V4B_Arm(HardwareMap map){
        left_servo = new Caching_Servo(map, "left_arm");
        right_servo = new Caching_Servo(map, "right_arm");
        release_servo = new Caching_Servo(map, "release_arm");
        front = new Caching_Servo(map, "frontGate");
        partialToggle = false;

        close();
    }

    public void start(){
        time.startTime();
    }

    public void reset(){
        left_servo.setPosition(0.969);
        right_servo.setPosition(0.014);
    }

    public void V4BOutPose(){
        left_servo.setPosition(0.047);
        right_servo.setPosition(1.0);
    }

    public void V4BLowGoalPose(){
        left_servo.setPosition(0.1);
        right_servo.setPosition(0.89);
    }

    public void V4BPartialOutPose(){
        left_servo.setPosition(0.3);
        right_servo.setPosition(0.725);
    }

    public void release(){
        release_servo.setPosition(0.69);
    }

    public void halfwayRelease(){
        release_servo.setPosition(0.52);

    }

    public void close(){
        release_servo.setPosition(0.45);
    }

    public void closeFront(){
        front.setPosition(0.7);
    }

    public void openFront(){
        front.setPosition(0.1);
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry){
        if(gamepad2.isPress(GamepadEx.Control.left_bumper)){
            lowGoal = false;
            out = !out;
        }

        if(gamepad.isPress(GamepadEx.Control.left_bumper)){
            lowGoal = true;
            out = !out;
        }

        if(gamepad.isPress(GamepadEx.Control.a) || gamepad2.isPress(GamepadEx.Control.a)){
            partialToggle = !partialToggle;
        }

        if(gamepad.isPress(GamepadEx.Control.right_bumper)){
            release = !release;
        }

        if(!out){
            release = false;
            close();
            openFront();
            reset();
            time.reset();
        } else{
            if(time.time() > 0.1) {
                if(time.time() > 0.2) {
                    if (!release) {
                        halfwayRelease();
                    } else {
                        release();
                    }
                }

                if (partialToggle) {
                    V4BPartialOutPose();
                } else {
                    if (lowGoal) {
                        V4BLowGoalPose();
                    } else {
                        V4BOutPose();
                    }
                }
            }

            closeFront();
        }

        telemetry.addData("Partial Toggle: ", partialToggle);
    }

    public void write(){
        left_servo.write();
        right_servo.write();
        release_servo.write();
        front.write();
    }
}
