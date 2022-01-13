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
    Caching_Servo release_servo;

    private boolean out = false;
    private boolean release = false;
    ElapsedTime time = new ElapsedTime();

    Caching_Servo front;

    boolean partialToggle = false;

    /*
        Max Out:
            Left: 0.985
            Right: 0.01

        Zero position(in):
            Right: 0.025
            Left: 0.95

        Mid Position:
            Right: 0.35
            Left: 0.67
     */

    public V4B_Arm(HardwareMap map){
        left_servo = new Caching_Servo(map, "left_arm");
        right_servo = new Caching_Servo(map, "right_arm");
        release_servo = new Caching_Servo(map, "release_arm");
        front = new Caching_Servo(map, "frontGate");
    }

    public void start(){
        time.startTime();
    }

    public void reset(){
        left_servo.setPosition(0.12);
        right_servo.setPosition(0.97);
    }

    public void V4BOutPose(){
        left_servo.setPosition(0.87);
        right_servo.setPosition(0.23);
    }

    public void V4BPartialOutPose(){
        left_servo.setPosition(0.669);
        right_servo.setPosition(0.4);
    }

    public void release(){
        release_servo.setPosition(0.55);
    }

    public void close(){
        release_servo.setPosition(0.425);
    }

    public void restRelease(){
        release_servo.setPosition(0.485);
    }

    public void closeFront(){
        front.setPosition(1.0);
    }

    public void openFront(){
        front.setPosition(0.8);
    }

    public void operate(GamepadEx gamepad, GamepadEx gamepad2, Telemetry telemetry){
        if(gamepad.isPress(GamepadEx.Control.left_bumper)){
            out = !out;
        }

        if(gamepad.isPress(GamepadEx.Control.a)){
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
            if(partialToggle) {
                V4BPartialOutPose();
            }else{
                V4BOutPose();
            }

            closeFront();

            if(!release){
                close();
            } else{
                release();
            }
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
