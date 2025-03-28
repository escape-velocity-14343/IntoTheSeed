package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.Objects;
import java.util.function.DoubleSupplier;

public class FineAlignCommand extends CommandBase {
    VisionSubsystem visionSubsystem;
    DefaultGoToPointCommand gtpc;
    MecanumDriveSubsystem drive;
    PinpointSubsystem pinpoint;
    TurretSubsystem turret;
    DoubleSupplier rotSupplier;
    ElapsedTime time = new ElapsedTime();
    ElapsedTime startTime = new ElapsedTime();
    private double distance = Double.MAX_VALUE;
    private double velocity = Double.MAX_VALUE;

    public FineAlignCommand(VisionSubsystem vision, DefaultGoToPointCommand gtpc, MecanumDriveSubsystem drive, PinpointSubsystem pinpoint, TurretSubsystem turret) {
        this.visionSubsystem = vision;
        this.gtpc = gtpc;
        this.drive = drive;
        this.pinpoint = pinpoint;
        this.turret = turret;
        this.rotSupplier = () ->
                Util.signedSqrt(Util.getAngularDifference(gtpc.getTargetHeading(), pinpoint.getPose().getRotation().getDegrees()) * DefaultGoToPointCommand.headingkP);
    }

    @Override
    public void initialize() {
        visionSubsystem.setCam(false);
        gtpc.setToggle(false);
        CommandScheduler.getInstance().schedule(new GoToPointWithDefaultCommand(gtpc.getCurrentPose(), gtpc));
        time.reset();
        startTime.reset();
    }

    @Override
    public void execute() {
        Pose2d pose = visionSubsystem.getSamplePose();

        double xMove, yMove, xDist, yDist;

        if (Objects.nonNull(pose)) {
            Log.v("FineAlign", "Sample X: " + pose.getX());
            Log.v("FineAlign", "Sample Y: " + pose.getY());

            xDist = pose.getX() - VisionConstants.xOffset;
            yDist = pose.getY() - VisionConstants.yOffset;

            xMove = xDist * VisionConstants.visionKP;
            yMove = yDist * VisionConstants.visionKP;

            Log.v("FineAlign", "XMove: " + xMove);
            Log.v("FineAlign", "YMove: " + yMove);

            turret.rotateTo(pose.getRotation().getDegrees());
            Log.v("FineAlign", "Sample Heading: " + pose.getHeading());
        } else {
            xMove = 0;
            yMove = 0;
            Log.v("FineAlign", "No samples detected");
            xDist = Double.MAX_VALUE;
            yDist = Double.MAX_VALUE;

        }

        double hMove = -rotSupplier.getAsDouble();

        // cap max move power
        double scalar = Math.hypot(xMove, yMove);

        if (scalar > VisionConstants.maxPower) {
            xMove *= VisionConstants.maxPower / scalar;
            yMove *= VisionConstants.maxPower / scalar;
        }

        double voltage = drive.getAutoVoltageMult();

        xMove *= voltage;
        yMove *= voltage;
        hMove *= voltage;

        if (VisionConstants.reverseX) {
            xMove *= -1;
        }

        if (VisionConstants.reverseY) {
            yMove *= -1;
        }

        // rc pid
        drive.driveFieldCentric(xMove, yMove, hMove, 0);

        distance = Math.hypot(xDist, yDist);
        velocity = pinpoint.getVelocity().getTranslation().getNorm() / time.seconds();
        time.reset();
    }

    @Override
    public boolean isFinished() {
        return distance < VisionConstants.visionEndThreshold ||
                (velocity < VisionConstants.velocityEnd && startTime.seconds() > 0.2);
    }

    @Override
    public void end(boolean interrupted) {
        gtpc.setTarget(pinpoint.getPose());
        gtpc.setToggle(true);
        Log.i("FineAlign", "Fine Align Done");
    }
}
