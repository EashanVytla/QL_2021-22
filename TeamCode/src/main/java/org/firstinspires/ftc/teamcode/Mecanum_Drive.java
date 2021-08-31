package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

public class Mecanum_Drive {
    DcMotor[] motors = new DcMotor[4];
    Telemetry telemetry;


    public Mecanum_Drive(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        motors[0] = map.get(DcMotor.class, "front_left");
        motors[1] = map.get(DcMotor.class, "front_right");
        motors[2] = map.get(DcMotor.class, "back_left");
        motors[3] = map.get(DcMotor.class, "back_right");

        motors[0].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double x, double y, double rot){
        double frontLeftMotorPower = y - x - rot;
        double frontRightMotorPower = y + x + rot;
        double backLeftMotorPower = y + x - rot;
        double backRightMotorPower = y - x + rot;

        double motorPowers[] = {Math.abs(frontLeftMotorPower),
                Math.abs(backRightMotorPower),
                Math.abs(backLeftMotorPower),
                Math.abs(frontRightMotorPower)};
        Arrays.sort(motorPowers);

        if(Math.abs(motorPowers[3]) > 1){
            frontLeftMotorPower /= Math.abs(motorPowers[3]);
            frontRightMotorPower /= Math.abs(motorPowers[3]);
            backRightMotorPower /= Math.abs(motorPowers[3]);
            backLeftMotorPower /= Math.abs(motorPowers[3]);
        }

        motors[0].setPower(frontLeftMotorPower);
        motors[1].setPower(frontRightMotorPower);
        motors[2].setPower(backLeftMotorPower);
        motors[3].setPower(backRightMotorPower);
    }

    public void setPower(double UpLeft, double BackLeft, double UpRight, double BackRight){
        motors[0].setPower(-UpLeft);
        motors[1].setPower(-UpRight);
        motors[2].setPower(-BackLeft);
        motors[3].setPower(-BackRight);
    }


    public void drive(Gamepad gamepad, double maxMove, double maxTurn){
        setPower(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x, -maxTurn, maxTurn));
    }
}

