package org.firstinspires.ftc.teamcode.OpModes;

import android.icu.util.Freezable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.Components.Slides;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class OrvilleAuto extends LinearOpMode {

    private enum State{
        DROP_INTAKE,
        PRE_LOAD_FREIGHT,
        CAROUSEL,
        RUN_CAROUSEL,
        DEPOT,
        CYCLE,
        CYCLE2,
        PARK,
        STOP
    }
    State mRobotState = State.DROP_INTAKE;

    int duckCase = 0;
    boolean gtp = false;
    int cycle = 0;
    int numCycles = 5;
    ElapsedTime time;
    Robot robot;
    double clearance_dist = 0.0;
    public static Pose2d PRE_LOADED_FREIGHT_POS = new Pose2d(22.156,20.433, Math.toRadians(180));
    public static Pose2d FREIGHT_POS = new Pose2d(6.78,26.46, Math.toRadians(215));
    public static Pose2d LOW_GOAL_FREIGHT_POS = new Pose2d(13.5, 20, Math.toRadians(217));
    public static Pose2d CAROUSEL_POS = new Pose2d(70,13, Math.toRadians(180));
    public static Pose2d DEPOT_POS = new Pose2d(-30.5,0, Math.toRadians(269));
    public static Pose2d PARK_POS = new Pose2d(-30,4, Math.toRadians(269));

    /*public static Pose2d PRE_LOADED_FREIGHT_POS = new Pose2d(22.156,20.433, Math.toRadians(180));
    public static Pose2d FREIGHT_POS = new Pose2d(6.78,26.46, Math.toRadians(215));
    public static Pose2d CAROUSEL_POS = new Pose2d(70,13, Math.toRadians(180));
    public static Pose2d DEPOT_POS = new Pose2d(-32.5,0.5, Math.toRadians(269));
    public static Pose2d PARK_POS = new Pose2d(-30,0.75, Math.toRadians(269));*/

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();

        robot.intake.clamp();
        robot.intake.write();

        robot.arm.closeFront();
        robot.arm.reset();
        robot.arm.close();
        robot.arm.write();
        telemetry.update();

        robot.localizer.reset();
        robot.setStartPose(new Pose2d(0, 0,Math.toRadians(90)));

        robot.initializeWebcam();

        while(!isStarted() && !isStopRequested()){
            duckCase = robot.getDuckCase();
            telemetry.addData("Duck Case", duckCase);
            telemetry.update();
        }

        robot.stopWebcam();

        waitForStart();

        time.reset();

        while(opModeIsActive()){
            ArrayList<CurvePoint> points = new ArrayList<>();

            switch(mRobotState) {
                case DROP_INTAKE:
                    if(time.time() >= 0.2){
                        robot.localizer.reset();
                        newState(State.PRE_LOAD_FREIGHT);
                    }

                    robot.intake.drop();
                    break;
                case PRE_LOAD_FREIGHT:
                    points.add(new CurvePoint(new Pose2d(0, 0, Math.toRadians(270)), 1d, 1d, 25));
                    points.add(new CurvePoint(duckCase == 0 ? LOW_GOAL_FREIGHT_POS : FREIGHT_POS, 1d, 1d, 25));

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) <= (duckCase == 0 ? 1 : 3.0) && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) <= (duckCase == 0 ? Math.toRadians(1) : 3.0)) {
                        if(time.time() > 0.25) {
                            newState(State.CYCLE2);
                        }
                        robot.arm.release();
                    } else if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 6.0){
                        time.reset();
                    }else {
                        if(duckCase == 2){
                            robot.slides.setPosition(157);
                        }

                        if(duckCase != 0){
                            robot.arm.V4BPartialOutPose();
                        }else{
                            robot.arm.V4BAutoLowGoalPos();
                        }
                    }
                    break;

                case CAROUSEL:
                    points.add(new CurvePoint(duckCase == 0 ? LOW_GOAL_FREIGHT_POS : FREIGHT_POS, 1d, 1d, 15));
                    points.add(new CurvePoint(new Pose2d(37.3, PRE_LOADED_FREIGHT_POS.getY() + 0.1, Math.PI), 1d, 1d, 15));
                    points.add(new CurvePoint(CAROUSEL_POS, 1d, 1d, 15));

                    if(robot.getPos().vec().distTo((duckCase == 0 ? LOW_GOAL_FREIGHT_POS : FREIGHT_POS).vec()) > 25){
                        robot.arm.close();
                        robot.arm.reset();
                        robot.arm.openFront();
                    }

                    if( robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 2) {
                        gtp = true;
                        time.reset();
                        newState(State.RUN_CAROUSEL);
                    }
                    break;
                case RUN_CAROUSEL:
                    points.add(new CurvePoint(new Pose2d(69.41, 10.5, Math.PI), 0.5d, 1d, 25));
                    if(time.time() > 3.5){
                        gtp = false;
                        robot.carousel.stopCarousel();
                        newState(State.DEPOT);
                    }else{
                        robot.carousel.runCarousel();
                    }
                    break;
                case DEPOT:
                    points.add(new CurvePoint(CAROUSEL_POS, 1d, 1d, 15));
                    points.add(new CurvePoint(new Pose2d(20, DEPOT_POS.getY() - 0.1, Math.toRadians(270)), 1d,1d,15));
                    points.add(new CurvePoint(DEPOT_POS, 1.0, 1d, 15));

                    if(robot.getPos().getX() < 0){
                        robot.intake.intake(true);
                    }

                    if( Math.abs(robot.getPos().getX() - DEPOT_POS.getX()) < 1.0) {
                        time.reset();
                        newState(State.CYCLE);
                    } else {
                        time.reset();
                    }
                    break;

                case CYCLE:
                    robot.slides.setBrake();

                    points.add(new CurvePoint(DEPOT_POS, 1d, 1d, 10));
                    points.add(new CurvePoint(new Pose2d(-20, clearance_dist, Math.toRadians(270)), 1d, 1d, 10)); //2nd POINT FOR FREIGHT_CYCLE
                    points.add(new CurvePoint(new Pose2d(0, clearance_dist, Math.toRadians(270)), 1d, 1d, 10)); //2nd POINT FOR FREIGHT_CYCLE

                    points.add(new CurvePoint(FREIGHT_POS, 1.0, 1.0, 10));


                    if(robot.getPos().getX() > -5) {
                        robot.arm.closeFront();
                        robot.slides.setPosition(185);
                        if(time.time() > 0.1) {
                            robot.arm.V4BPartialOutPose();
                        }
                        robot.intake.stop();
                    }else if(robot.getPos().getX() > -27){
                        robot.intake.intake(false);
                    }

                    if (robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 3.0) {
                        if(time.time() > 0.25) {
                            clearance_dist += 0.35;
                            DEPOT_POS = new Pose2d(DEPOT_POS.getX(), DEPOT_POS.getY() + 0.35, DEPOT_POS.getHeading());
                            newState(State.CYCLE2);
                        }
                        robot.arm.release();
                    }else{
                        if(robot.getPos().getX() > 0) {
                            time.reset();
                        }
                    }
                    break;

                case CYCLE2:
                    points.add(new CurvePoint(FREIGHT_POS, 1d, 1d, 10));
                    points.add(new CurvePoint(new Pose2d(7, clearance_dist, Math.toRadians(270)), 1d, 1d, 10)); //2nd POINT FOR FREIGHT_CYCLE //2nd POINT FOR FREIGHT_CYCLE
                    if(cycle <= numCycles) {
                        if(cycle >= 3){
                            points.add(new CurvePoint(DEPOT_POS, 1d, 1d, 10));
                            points.add(new CurvePoint(new Pose2d(DEPOT_POS.getX() + 0.1 - cycle * 2.0, DEPOT_POS.getY() + cycle * 1.0, DEPOT_POS.getHeading() + Math.toRadians(30)), 1d, 1d, 10));
                        }else{
                            points.add(new CurvePoint(new Pose2d(DEPOT_POS.getX() - cycle * 3.0, DEPOT_POS.getY(), DEPOT_POS.getHeading()), 1d, 1d, 10));
                        }
                    }else{
                        points.add(new CurvePoint(PARK_POS, 1d, 1d, 10));
                    }

                    if(robot.getPos().getX() < 0) {
                        robot.intake.intake(true);
                    }

                    if(cycle != 0 || robot.getPos().vec().distTo((duckCase == 0 ? LOW_GOAL_FREIGHT_POS : FREIGHT_POS).vec()) > 10) {
                        robot.arm.close();
                        robot.arm.reset();
                        robot.arm.openFront();

                        robot.slides.setCoast();
                        if(robot.slides.isDown()){
                            telemetry.addLine("down");
                            robot.slides.setPower(0.0);
                        }else{
                            robot.slides.setPower(-robot.slides.downPower);
                        }
                    }


                    if (robot.getPos().getX() - 1 < points.get(points.size() - 1).x && Math.abs(robot.getPos().getHeading() - points.get(points.size() - 1).heading) < 2.0) {
                        if(cycle <= numCycles) {
                            FREIGHT_POS = new Pose2d(FREIGHT_POS.getX(), FREIGHT_POS.getY() + 0.35, FREIGHT_POS.getHeading());
                            cycle++;
                            newState(State.CYCLE);
                        } else {
                            newState(State.PARK);
                        }
                    }else {
                        time.reset();
                    }

                    break;
                case PARK:
                    gtp = true;
                    points.add(new CurvePoint(PARK_POS, 1d, 1d, 10));
                    robot.intake.stop();

                    break;
                case STOP:
                    robot.drive.setCoast();
                    robot.drive.setPower(0, 0, 0);
                    robot.drive.write();
                    robot.updatePos();
                    break;
            }

            if(mRobotState != State.STOP) {
                if (points.size() != 0) {
                    if (!gtp) {
                        RobotMovement.followCurve(points, robot, telemetry);
                    } else {
                        robot.GoTo(points.get(points.size() - 1).toPose(), new Pose2d(points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).moveSpeed, points.get(points.size() - 1).turnSpeed));
                    }
                } else {
                    robot.drive.setPower(0, 0, 0);
                    robot.drive.write();
                    robot.updatePos();
                }
            }

            telemetry.addData("State", mRobotState);
            telemetry.addData("Position", robot.getPos());
            telemetry.addData("Depot Pos", DEPOT_POS.toString());
            telemetry.addData("Clearance Dist", clearance_dist);
            telemetry.update();
            robot.arm.write();
            robot.slides.write();
            robot.carousel.write();
            robot.intake.write();
        }
    }

    public void newState(State state){
        time.reset();
        mRobotState = state;
    }
}