package org.firstinspires.ftc.teamcode.Components;


import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.Vector2;
import org.firstinspires.ftc.teamcode.Wrapper.Caching_Motor;

import java.util.Arrays;

@Config
public class Mecanum_Drive{
    Caching_Motor[] motors = new Caching_Motor[4];

    Telemetry telemetry;

    PIDFController PID_X;
    PIDFController PID_Y;
    PIDFController PID_Z;

    public static double kp = 0.12;
    public static double ki = 0;
    public static double kd = 0.011;

    public static double kpr = 2.5;
    public static double kir = 0;
    public static double kdr = 0.15;

    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static boolean isTurning = false;
    private boolean blue;

    int counter;

    public Mecanum_Drive(HardwareMap map, Telemetry telemetry){
        this.telemetry = telemetry;
        motors[0] = new Caching_Motor(map, "front_left");
        motors[1] = new Caching_Motor(map, "front_right");
        motors[2] = new Caching_Motor(map, "back_left");
        motors[3] = new Caching_Motor(map, "back_right");

        motors[0].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[1].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[2].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors[3].motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motors[0].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PID_X = new PIDFController(new PIDCoefficients(kp, ki, kd));
        PID_Y = new PIDFController(new PIDCoefficients(kp, ki, kd));
        PID_Z = new PIDFController(new PIDCoefficients(kpr, kir, kdr));
        counter = 0;

        motors[1].motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].motor.setDirection(DcMotorSimple.Direction.REVERSE );
    }

    public void setBlue(){
        blue = true;
    }

    public void setCoast(){
        motors[0].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[1].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[2].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motors[3].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void setBrake(){
        motors[0].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[1].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[2].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motors[3].motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void write(){
        motors[counter].write();
        counter = (counter + 1) % 4;
    }

    public void setPower(double x, double y, double rot){
        isTurning = rot > 0.05 && x == 0 && y == 0;
        telemetry.addData("Is turning", isTurning);

        //Calculating the 4 motor powers
        double frontLeftMotorPower = y - x - rot;
        double frontRightMotorPower = y + x + rot;
        double backLeftMotorPower = y + x - rot;
        double backRightMotorPower = y - x + rot;

        //Creating an array of all motor powers
        double motorPowers[] = {Math.abs(frontLeftMotorPower),
                Math.abs(backRightMotorPower),
                Math.abs(backLeftMotorPower),
                Math.abs(frontRightMotorPower)};
        //Sorting the motor powers to find the highest one
        Arrays.sort(motorPowers);

        //Verifying highest motor power != 0 to avoid a divide by 0 error
        if(Math.abs(motorPowers[3]) > 1){
            //Normalizing the values to be within 0-1
            frontLeftMotorPower /= Math.abs(motorPowers[3]);
            frontRightMotorPower /= Math.abs(motorPowers[3]);
            backRightMotorPower /= Math.abs(motorPowers[3]);
            backLeftMotorPower /= Math.abs(motorPowers[3]);
        }

        //Setting the powers
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

    public void setPower(Vector2 vec, double rot){
        setPower(vec.x, vec.y, rot);
    }

    public void setPowerCentic(double x, double y, double rot, double heading){
        if(blue) {
            heading -= Math.PI;
            setPower(new Vector2(x, y).rotated((heading) + Math.PI), rot);
        }else{
            setPower(new Vector2(x, y).rotated(((2 * Math.PI) - heading) + Math.PI), rot);
        }
    }

    public void drive(Gamepad gamepad, double scale, double turnScale, double maxMove, double maxTurn){
        setPower(Range.clip(gamepad.left_stick_x * scale, -maxMove, maxMove), Range.clip(gamepad.left_stick_y * scale, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x * turnScale, -maxTurn, maxTurn));
    }

    public void drive(Gamepad gamepad, double maxMove, double maxTurn){
        setPower(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x, -maxTurn, maxTurn));
    }

    public void driveCentric(Gamepad gamepad, double maxMove, double maxTurn, double heading){
        if(blue) {
            setPowerCentic(Range.clip(-gamepad.left_stick_x, -maxMove, maxMove), Range.clip(-gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x, -maxTurn, maxTurn), heading);
        }else{
            setPowerCentic(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x, -maxTurn, maxTurn), heading);
        }
    }

    public void driveCentric(Gamepad gamepad, double turnScale, double maxMove, double maxTurn, double heading){
        setPowerCentic(Range.clip(gamepad.left_stick_x, -maxMove, maxMove), Range.clip(gamepad.left_stick_y, -maxMove, maxMove), Range.clip(-gamepad.right_stick_x * turnScale, -maxTurn, maxTurn), heading);
    }

    public void goToPoint(Pose2d targetPos, Pose2d currentPos, double xspeed, double yspeed, double zspeed){
        PID_X.setOutputBounds(-xspeed, xspeed);
        PID_Y.setOutputBounds(-yspeed, yspeed);
        PID_Z.setOutputBounds(-zspeed, zspeed);

        double heading = 0;
        double target_heading = targetPos.getHeading();

        if(currentPos.getHeading() <= Math.PI){
            heading = currentPos.getHeading();
        }else{
            heading = -((2 * Math.PI ) - currentPos.getHeading());
        }

        if(Math.abs(targetPos.getHeading() - heading) >= Math.toRadians(180.0)){
            target_heading = -((2 * Math.PI) - targetPos.getHeading());
        }

        telemetry.addData("Target Pos: ", targetPos);
        telemetry.addData("Current Pos: ", currentPos);
        telemetry.addData("Translational Error", targetPos.vec().distTo(currentPos.vec()));
        telemetry.addData("Rotational Error", Math.toDegrees(Math.abs(target_heading - heading)));

        PID_X.setTargetPosition(targetPos.getX());
        PID_Y.setTargetPosition(targetPos.getY());
        PID_Z.setTargetPosition(target_heading);

        if(!blue) {
            setPowerCentic(PID_X.update(currentPos.getX()), -PID_Y.update(currentPos.getY()), -PID_Z.update(heading), currentPos.getHeading());
        }else {
            setPowerCentic(PID_X.update(currentPos.getX()), PID_Y.update(currentPos.getY()), PID_Z.update(heading), currentPos.getHeading());
        }
    }
}
