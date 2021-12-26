package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Slide_Tester extends OpMode {
    Caching_Motor rSlides;
    Caching_Motor lSlides;
    V4B_Arm arm;


    public void init(){
        rSlides = new Caching_Motor(hardwareMap, "rSlide");
        lSlides = new Caching_Motor(hardwareMap, "lSlide");
        arm = new V4B_Arm(hardwareMap);
    }

    public void loop(){
        if(gamepad1.right_stick_y >= 0) {
            rSlides.setPower((gamepad1.right_stick_y) + 0.25);
            lSlides.setPower(-(gamepad1.right_stick_y) - 0.25);
        } else {
            rSlides.setPower((gamepad1.right_stick_y*0.3) + 0.25);
            lSlides.setPower(-(gamepad1.right_stick_y*0.3) - 0.25);
        }
        rSlides.write();
        lSlides.write();
        arm.reset();
        arm.write();

        telemetry.addData("rSlide", rSlides.motor.getPower());
        telemetry.addData("lSlide", lSlides.motor.getPower());
        telemetry.addData("GamepadPower", gamepad1.right_stick_y);
    }
}
