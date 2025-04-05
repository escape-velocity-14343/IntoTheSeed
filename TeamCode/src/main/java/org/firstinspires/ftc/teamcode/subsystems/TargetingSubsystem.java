package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cx;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cy;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.focalL;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;

public class TargetingSubsystem extends SubsystemBase {
    private final VisionSubsystem vision;
    private final PinpointSubsystem pinpoint;
    RobotPnP pnp = new RobotPnP(cx, cy, focalL);
    Translation2d target = new Translation2d(-12, 12); //default value
    private double angle = 0;
    private Telemetry telemetry;
    private double targetToIVK = 10.0;

    public TargetingSubsystem(VisionSubsystem vision, PinpointSubsystem pinpoint, Telemetry telemetry){
        this.pinpoint = pinpoint;
        this.vision = vision;
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Target pos X", target.getX());
        telemetry.addData("Target pos Y", target.getY());
        telemetry.addData("Target Angle", angle);
    }

    public void cycle() {
        if (vision.getClosestChassis().isPresent()){
            VisionSubsystem.sample sample = vision.getClosestChassis().get();
            angle = sample.angle;
            target = pnp.getFieldCoordinates((int)sample.x, (int)sample.y, pinpoint.getPose());
        }
    }

    public Translation2d getTarget(){
        return target;
    }

    public double getX(){
        return target.getX();
    }

    public double getY(){
        return target.getY();
    }

    public double getAngle(){
        return angle;
    }

    public Pose2d getDBTarget(){
        return new Pose2d(this.getX(), AutoConstants.subBarrierY, Rotation2d.fromDegrees(-90));
    }

    public double getIVKY(){
        telemetry.addData("IVKY", pinpoint.getPose().getTranslation().minus(target).getY());
        Log.i("IVKY", pinpoint.getPose().getTranslation().minus(target).toString());
        return pinpoint.getPose().getTranslation().minus(target).getY();
    }

    public Translation2d sampleFieldRelative(){
        return pinpoint.getPose().getTranslation().plus(target);
    }
}
