package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.Components.V4B_Arm;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

@TeleOp
public class Slide_Tester extends OpMode {
    Slides slides;

    public void init(){
        slides = new Slides(hardwareMap, telemetry);
    }

    public void loop(){
        telemetry.addData("Position", slides.getPosition());

        slides.write();
        telemetry.update();
    }
}
