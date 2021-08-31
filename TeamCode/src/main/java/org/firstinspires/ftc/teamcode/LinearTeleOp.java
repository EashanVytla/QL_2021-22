package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Mecanum_Drive drive;
    DcMotor intake;

    @Override
    public void runOpMode() {
        drive = new Mecanum_Drive(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            drive.drive(gamepad1, 1.0, 1.0);
        }
    }
}