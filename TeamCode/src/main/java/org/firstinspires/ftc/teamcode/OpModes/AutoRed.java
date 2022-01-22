package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Robot;
import org.firstinspires.ftc.teamcode.PurePusuit.CurvePoint;
import org.firstinspires.ftc.teamcode.PurePusuit.RobotMovement;

import java.util.ArrayList;

@Autonomous
public class AutoRed extends LinearOpMode {

    private enum State{
        PRE_LOAD_FREIGHT,
        CAROUSEL,
        FREIGHT_CYCLE,
        PARK
    }
    State mRobotState = State.PRE_LOAD_FREIGHT;

    boolean gtp = false;
    ElapsedTime time;
    Robot robot;
    private Pose2d INITIAL_POS = new Pose2d(0,0, Math.toRadians(0));
    private Pose2d PRE_LOAD_FREIGHT_POS = new Pose2d(0,0, Math.toRadians(0));
    private Pose2d CAROUSEL_POS = new Pose2d(0,0, Math.toRadians(0));
    private Pose2d FREIGHT_CYCLE_POS = new Pose2d(0,0, Math.toRadians(0));
    private Pose2d PARK_POS = new Pose2d(0,0, Math.toRadians(0));

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        time = new ElapsedTime();
        robot.localizer.reset();

        //Need Position
        robot.setStartPose(new Pose2d(0, 0,0));

        while(!isStarted() && !isStopRequested()){
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            ArrayList<CurvePoint> points = new ArrayList<>();
            switch(mRobotState){
                case PRE_LOAD_FREIGHT:
                    points.add(new CurvePoint(INITIAL_POS, 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-16d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(PRE_LOAD_FREIGHT_POS, 1d, 1d, 25));


                    if(robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 0.1){
                        if(time.time() > 2.0){
                            newState(State.CAROUSEL);
                        }
                    } else {
                        time.reset();
                    }
                    break;

                case CAROUSEL:
                    points.add(new CurvePoint(new Pose2d(-16d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(CAROUSEL_POS, 1d, 1d, 25));


                    if( robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 0.1) {
                        if(time.time() > 2.0){
                            newState(State.FREIGHT_CYCLE);
                        }
                    } else {
                        time.reset();
                    }
                    break;

                case FREIGHT_CYCLE:
                    points.add(new CurvePoint(new Pose2d(-16d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(new Pose2d(-13d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(FREIGHT_CYCLE_POS, 1d, 1d, 25));


                    if( robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 0.1) {
                        if (time.time() > 2.0) {
                            newState(State.PARK);
                        }
                    } else {
                        time.reset();
                    }
                    break;

                case PARK:
                    points.add(new CurvePoint(new Pose2d(-16d, 27d, Math.toRadians(0)), 1d, 1d, 25));
                    points.add(new CurvePoint(PARK_POS, 1d, 1d, 25));

                    if( robot.getPos().vec().distTo(points.get(points.size() - 1).toVec()) < 0.1) {
                        robot.drive.setPower(0,0,0);
                    }
                    break;
            }

            if(points.size() != 0){
                if(!gtp){
                    RobotMovement.followCurve(points, robot, telemetry);
                }else{
                    robot.GoTo(points.get(points.size() - 1).toPose(),new Pose2d(1, 1, 1));
                }
            }else{
                robot.drive.setPower(0, 0, 0);
                robot.updatePos();
            }
        }
        telemetry.addData("State", mRobotState);
        telemetry.addData("Position", robot.getPos());
    }

    public void newState(State state){
        robot.localizer.reset();
        mRobotState = state;
    }
}
