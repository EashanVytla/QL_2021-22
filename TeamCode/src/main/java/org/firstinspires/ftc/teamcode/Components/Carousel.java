package org.firstinspires.ftc.teamcode.Components;

import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Carousel {
    CRServo left;
    CRServo right;
    Telemetry telemetry;

    public Carousel(HardwareMap map, Telemetry telemetry){
        left = map.get(CRServo.class, "carousel_left");
        right = map.get(CRServo.class, "carousel_right");
    }

    public void write(){

    }

    public void runCarousel(){
        if(Robot.red) {
            left.setPower(0.5);
        }else{
            right.setPower(0.5);
        }
    }

    public void operate(Gamepad gamepad){
        if(gamepad.dpad_left){
            left.setPower(1);
        }else{
            left.setPower(0);
        }

        if(gamepad.dpad_right){
            right.setPower(1);
        }else{
            right.setPower(0);
        }
    }

    public void stopCarousel(){
        if(Robot.red) {
            left.setPower(0);
        }else{
            right.setPower(0);
        }
    }
}
