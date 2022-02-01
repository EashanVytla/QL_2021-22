package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;

import java.util.ArrayList;

@TeleOp(name = "TeleOp")
public class LinearTeleOp extends LinearOpMode {
    Robot robot;
    GamepadEx gamepadEx1;
    GamepadEx gamepadEx2;
    int toggle = 0;

    enum mRobotState{
        AUTOMATION,
        DRIVE,
    }

    enum mAutomationState{
        CYCLE,
        CYCLE2,
    }

    mRobotState robotState = mRobotState.DRIVE;
    mAutomationState automationState = mAutomationState.CYCLE;


    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);
        ElapsedTime time = new ElapsedTime();

        waitForStart();

        robot.arm.start();

        while (opModeIsActive()) {
            switch (robotState){
                case DRIVE:
                    robot.operate(gamepadEx1, gamepadEx2);
                    telemetry.update();

                    if(gamepadEx1.isPress(GamepadEx.Control.b)){
                        time.reset();
                        robotState = mRobotState.AUTOMATION;
                        automationState = mAutomationState.CYCLE;
                    }
                    break;
                case AUTOMATION:
                    ArrayList<CurvePoint> points = new ArrayList<>();

                    if(gamepadEx2.isPress(GamepadEx.Control.dpad_up)){
                        toggle++;
                    }else{
                        toggle--;
                    }

                    switch (automationState) {
                        case CYCLE:
                            robot.slides.setBrake();
                            points.add(new CurvePoint(AutoRed.DEPOT_POS, 1d, 1d, 15));
                            points.add(new CurvePoint(new Pose2d(5, 0, Math.toRadians(270)), 1d, 1d, 15)); //2nd POINT FOR FREIGHT_CYCLE
                        /*switch(cycle){
                            case 0:
                            case 3:
                                points.add(new CurvePoint(FREIGHT_POS, 1d, 1d, 15));
                                break;
                            case 1:
                                points.add(new CurvePoint(FREIGHT_POS_2, 1d, 1d, 15));
                                break;
                            case 2:
                                points.add(new CurvePoint(FREIGHT_POS_3, 1d, 1d, 15));
                                break;
                        }*/
                            points.add(new CurvePoint(AutoRed.FREIGHT_POS, 0.5d, 0.5d, 15));

                            if (robot.getPos().getX() > -5) {
                                robot.arm.closeFront();
                                robot.slides.setPosition(157);
                                robot.arm.V4BPartialOutPose();
                                robot.intake.stop();
                            } else if (robot.getPos().getX() <= 0) {
                                if (time.time() > 0.5) {
                                    robot.intake.intake(false);
                                }
                            }

                            if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2 && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < 2.0) {
                                if (time.time() > 0.2) {
                                    newState(mAutomationState.CYCLE2);
                                }
                                robot.arm.release();
                            } else {
                                if (robot.getPos().getX() > 0) {
                                    time.reset();
                                }
                            }
                            break;

                        case CYCLE2:
                            points.add(new CurvePoint(AutoRed.FREIGHT_POS, 1d, 1d, 10));
                            points.add(new CurvePoint(new Pose2d(5, -1, Math.toRadians(270)), 1d, 1d, 10)); //2nd POINT FOR FREIGHT_CYCLE
                            points.add(new CurvePoint(new Pose2d(AutoRed.DEPOT_POS.getX() - toggle * 4, AutoRed.DEPOT_POS.getY(), AutoRed.DEPOT_POS.getHeading()), 1d, 1d, 10));


                            if (robot.getPos().getX() < 0) {
                                robot.intake.intake(true);
                            }

                            robot.arm.close();
                            robot.arm.reset();
                            robot.arm.openFront();

                            robot.slides.setCoast();
                            if (robot.slides.getPosition() < 50) {
                                if (!robot.slides.isDown()) {
                                    telemetry.addLine("not down");
                                    robot.slides.setPower(-0.2);
                                } else {
                                    telemetry.addLine("down");
                                    robot.slides.setPower(-0.03);
                                }
                            } else {
                                robot.slides.setPower(-robot.slides.downPower);
                            }

                            if (robot.getPos().getX() - 1 < points.get(points.size() - 1).x) {
                                newState(mAutomationState.CYCLE);
                            } else {
                                time.reset();
                            }

                            break;
                    }
                    RobotMovement.followCurve(points, robot, telemetry);

                    break;
            }
        }
    }

    public void newState(mAutomationState state){
        automationState = state;
    }
}