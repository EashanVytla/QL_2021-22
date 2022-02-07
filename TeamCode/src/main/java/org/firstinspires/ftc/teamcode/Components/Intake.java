package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.IterationListener;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Servo;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    Caching_Motor intake;
    Caching_Servo intake_dropper;
    double power = 1.0;
    boolean intakeToggle = false;

    public Intake(HardwareMap map){
        intake = new Caching_Motor(map, "intakeL");
        intake_dropper = new Caching_Servo(map, "intake_dropper");
        intake.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void intake(GamepadEx gamepadEx, GamepadEx gamepad2Ex){
        /*intakeR.setPower(gamepadEx.gamepad.right_trigger - gamepadEx.gamepad.left_trigger);
        intakeL.setPower(-gamepadEx.gamepad.right_trigger + gamepadEx.gamepad.left_trigger);*/

        clamp();
        if(gamepadEx.isPress(GamepadEx.Control.right_trigger)){
            intakeToggle = !intakeToggle;
        }

        if(Math.abs(gamepad2Ex.gamepad.left_trigger + gamepad2Ex.gamepad.right_trigger) > 0.1) {
            intake.setPower(gamepad2Ex.gamepad.left_trigger + gamepad2Ex.gamepad.right_trigger);
        }else{
            if(Math.abs(gamepadEx.gamepad.left_trigger) < 0.1) {
                if (intakeToggle) {
                    intake.setPower(-1.0);
                }else{
                    intake.setPower(0.0);
                }
            }else{
                intake.setPower(0.5);
            }
        }

    }

    public void intake(boolean ifTrue){
        if(ifTrue){
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.5);
        }
    }

    public void stop(){
        intake.setPower(0.0);
    }

    public void clamp(){
        intake_dropper.setPosition(0.08);
    }

    public void drop(){
        intake_dropper.setPosition(0.05);
    }

    public void write(){
        intake.write();
        intake_dropper.write();
    }

}
