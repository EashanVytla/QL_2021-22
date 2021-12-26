package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Encoder;
import org.firstinspires.ftc.teamcode.Odometry.S4T_Localizer;
import org.firstinspires.ftc.teamcode.Wrapper.GamepadEx;
import java.util.List;

public class Robot {
    public Mecanum_Drive drive;
    public V4B_Arm arm;
    public Intake intake;
    public S4T_Localizer localizer;
    private S4T_Encoder encoderLY;
    private S4T_Encoder encoderLX;
    private S4T_Encoder encoderRY;
    private S4T_Encoder encoderRX;
    private HardwareMap hardwareMap;

    private Telemetry telemetry;

    private static Pose2d startPos = new Pose2d(0, 0, 0);
    List<LynxModule> allHubs;

    public Robot(HardwareMap map, Telemetry telemetry){
        this.hardwareMap = map;
        this.telemetry = telemetry;

        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        encoderLY = new S4T_Encoder(map, "back_left");
        encoderLX = new S4T_Encoder(map, "front_left");
        encoderRY = new S4T_Encoder(map, "back_right");
        encoderRX = new S4T_Encoder(map, "front_right");

        drive = new Mecanum_Drive(map, telemetry);
        arm = new V4B_Arm(map);
        intake = new Intake(map);

        updateBulkData();

        localizer = new S4T_Localizer(hardwareMap, telemetry);
    }

    public void operate(GamepadEx gamepad1ex, GamepadEx gamepad2ex) {
        updateBulkData();
        updatePos();

        drive.drive(gamepad1ex.gamepad, 1.0, 1.0, 0.6, 1.0/*, getPos().getHeading() + Math.toRadians(90)*/);

        intake.intake(gamepad1ex, telemetry);

        arm.operate(gamepad1ex, gamepad2ex);

        arm.write();
        intake.write();
        drive.write();

        telemetry.addData("Robot Position:", getPos());
        gamepad1ex.loop();
    }

    public void setStartPose(Pose2d startPos){
        this.startPos = startPos;
    }

    public void updateBulkData(){
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
    }

    public void updatePos(){
        encoderLX.update();
        encoderLY.update();
        encoderRX.update();
        encoderRY.update();
        localizer.update(getRawLeft_X_Dist(), getRawLeft_Y_Dist(), getRawRight_X_Dist(), getRawRight_Y_Dist());
    }

    public double getLeft_X_Dist(){
        return encoderLX.getDist();
    }

    public double getRight_X_Dist(){
        return encoderRX.getDist();
    }

    public double getLeft_Y_Dist(){
        return encoderLY.getDist();
    }

    public double getRight_Y_Dist(){
        return encoderRY.getDist();
    }

    public double getRawLeft_X_Dist(){
        return encoderLX.distance;
    }

    public double getRawRight_X_Dist(){
        return encoderRX.distance;
    }

    public double getRawLeft_Y_Dist(){
        return encoderLY.distance;
    }

    public double getRawRight_Y_Dist(){
        return encoderRY.distance;
    }

    public Pose2d getPos(){
        return new Pose2d(localizer.getPose().getX() + startPos.getX(), localizer.getPose().getY() + startPos.getY(), localizer.getPose().getHeading() + startPos.getHeading());
    }

    public Pose2d getStartPos(){
        return startPos;
    }

    public void GoTo(Pose2d pose, Pose2d speedLimits){
        updateGoTo(pose, speedLimits);
    }

    public void GoTo(double x, double y, double heading, double maxspeed_x, double maxspeed_y, double maxspeed_z){
        updateGoTo(new Pose2d(x, y, heading), new Pose2d(maxspeed_x, maxspeed_y, maxspeed_z));
    }

    public void setAngle(double heading){
        localizer.setHeading(heading);
    }

    private void updateGoTo(Pose2d pose, Pose2d speedLimits){
        drive.goToPoint(pose, getPos(), speedLimits.getX(), speedLimits.getY(), speedLimits.getHeading());
        telemetry.addData("Position: ", getPos());
        telemetry.addData("Target Position: ", pose);
        drive.write();
        updatePos();
    }
}
