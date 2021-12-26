package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.IterationListener;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

public class Intake {
    Caching_Motor intakeR;
    Caching_Motor intakeL;
    double power = 1.0;
    boolean intakeToggle = false;

    public Intake(HardwareMap map){
        intakeR = new Caching_Motor(map, "intakeR");
        intakeL = new Caching_Motor(map, "intakeL");
    }

    public void intake(GamepadEx gamepadEx, Telemetry telemetry){
        /*intakeR.setPower(gamepadEx.gamepad.right_trigger - gamepadEx.gamepad.left_trigger);
        intakeL.setPower(-gamepadEx.gamepad.right_trigger + gamepadEx.gamepad.left_trigger);*/

        if(gamepadEx.isPress(GamepadEx.Control.right_trigger)){
            intakeToggle = !intakeToggle;
        }

        if(Math.abs(gamepadEx.gamepad.left_trigger) < 0.1) {
            if (intakeToggle) {
                intakeR.setPower(1.0);
                intakeL.setPower(-1.0);
            }else{
                intakeL.setPower(0.0);
                intakeR.setPower(0.0);
            }
        }else{
            intakeR.setPower(-0.5);
            intakeL.setPower(0.5);
        }

        telemetry.addData("Power", power);
    }

    public void write(){
        intakeR.write();
        intakeL.write();
    }

}
