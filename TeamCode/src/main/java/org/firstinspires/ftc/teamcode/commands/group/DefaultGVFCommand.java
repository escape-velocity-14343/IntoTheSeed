package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.path.follower.GVFFollower;
import org.firstinspires.ftc.teamcode.lib.path.spline.Spline;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

import java.util.ArrayList;

@Config
public class DefaultGVFCommand extends CommandBase {
    public GVFFollower gvf = new GVFFollower();

    public double tol = 3;
    public double hTol = 4;

    MecanumDriveSubsystem drive;
    PinpointSubsystem pinpoint;

    private boolean toggle = true;
    private double distanceToEnd = 0.0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime zeroVelocityTimer = new ElapsedTime();

    private boolean isZeroVelocity = false;
    private boolean hasBeenZeroVelocity = false;
    private final ArrayList<DistanceAndCommand> whenCloseCommands = new ArrayList<>();
    private double endingDistance = -1.0;

    Pose2d target;
    Pose2d currentPose;

    public DefaultGVFCommand(
            MecanumDriveSubsystem driveSubsystem,
            PinpointSubsystem otosSubsystem,
            double hTol,
            Spline... splines) {
        this(driveSubsystem, otosSubsystem, splines);
        this.hTol = hTol;
    }

    public DefaultGVFCommand(
            MecanumDriveSubsystem driveSubsystem,
            PinpointSubsystem otosSubsystem,
            Spline... splines) {
        drive = driveSubsystem;
        pinpoint = otosSubsystem;
        gvf.setSplines(splines);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        currentPose = pinpoint.getPose();
        if (currentPose == null) {
            Log.i("execute", "currentPose was null (why?????)");
        }

    }

    @Override
    public void execute() {
        currentPose = pinpoint.getPose();
        target = gvf.getEndpoint();
        if (currentPose == null) {
            Log.w("execute", "currentPose was null (why?????)");
        } else {
            // Log.i("execute", "current pose was NOT null");
        }

        double xDist = target.getX() - currentPose.getX();
        double yDist = target.getY() - currentPose.getY();
        Log.v("GVF", "xDist: " + xDist + ", yDist: " + yDist);
        distanceToEnd = Math.sqrt(xDist * xDist + yDist * yDist);

        for (DistanceAndCommand whenCloseCommand : whenCloseCommands) {
            if (distanceToEnd < whenCloseCommand.distance) {
                CommandScheduler.getInstance().schedule(whenCloseCommand.command);
                whenCloseCommands.remove(whenCloseCommand);
            }
        }

        Pose2d move = gvf.update(currentPose, pinpoint.getVelocity());

        double xMove = move.getX();
        double yMove = move.getY();

        Log.v("GVF", "target: (" + getTargetX() + ", " + getTargetY() + ")");
        Log.v("GVF", "attempted movement: (" + xMove + ", " + yMove + ")");

        double voltageScalar = drive.getAutoVoltageMult();

        xMove *= voltageScalar;
        yMove *= voltageScalar;
        double hMove = move.getRotation().getDegrees();

        if (toggle) {
            drive.driveFieldCentric(-xMove, -yMove * 1.2, hMove);
        }

        // velocity end
        if (pinpoint.getVelocity().getTranslation().getNorm() < AutoConstants.stallVelocity) {
            if (!isZeroVelocity) {
                zeroVelocityTimer.reset();
                isZeroVelocity = true;
                hasBeenZeroVelocity = false;
            } else if (zeroVelocityTimer.seconds() > 1.0) {
                hasBeenZeroVelocity = true;
            } else {
                hasBeenZeroVelocity = false;
            }
        } else {
            isZeroVelocity = false;
        }
    }

    public void setToggle(boolean toggle) {
        this.toggle = toggle;
    }

    @Override
    public boolean isFinished() {
        return endingDistance > 0 && distanceToEnd < endingDistance;
    }

    @Override
    public void end(boolean wasInterrupted) {
        if (wasInterrupted) {
            drive.driveFieldCentric(0, 0, 0);
            Log.w("%Commands", "gvfc was interrupted. This should never happen!");
        } else {
            Log.w("%GVF", "this ended without interruption somehow???");
        }
    }

    public boolean isDone() {
        if (target == null) {
            Log.i("%isDone", "target was null");
            return false;
        }

        if (currentPose == null) {
            Log.i("%isDone", "currentPose was null");
            return false;
        }

        return ((currentPose.getTranslation().getDistance(target.getTranslation())
                < tol)
                && (Util.inRange(
                target.getRotation().getDegrees(),
                currentPose.getRotation().getDegrees(),
                hTol)))
                || (hasBeenZeroVelocity);
    }

    public void setSplines(Spline... splines) {
        this.gvf.setSplines(splines);

        isZeroVelocity = false;
        zeroVelocityTimer.reset();
        hasBeenZeroVelocity = false;
        timer.reset();
    }

    public void setSplines(boolean reverseHeading, Spline... splines) {
        this.gvf.setSplines(reverseHeading, splines);

        isZeroVelocity = false;
        zeroVelocityTimer.reset();
        hasBeenZeroVelocity = false;
        timer.reset();
    }

    public double getTargetHeading() {
        return target.getRotation().getDegrees();
    }

    public double getTargetX() {
        return target.getX();
    }

    public double getTargetY() {
        return target.getY();
    }

    public Pose2d getCurrentPose() {
        return currentPose;
    }

    public Translation2d getTranslation() {
        return currentPose.getTranslation();
    }

    public void setTolerances(double tol, double hTol) {
        this.tol = tol;
        this.hTol = hTol;
    }

    public DefaultGVFCommand whenClose(Command command, double distance) {
        whenCloseCommands.add(new DistanceAndCommand(command, distance));
        return this;
    }

    public DefaultGVFCommand setTangentOffset(double offset) {
        gvf.setTangentOffset(offset);
        return this;
    }

    public DefaultGVFCommand endWhenClose(double distance) {
        endingDistance = distance;
        return this;
    }

    private class DistanceAndCommand {
        public Command command;
        public double distance;

        public DistanceAndCommand(Command command, double distance) {
            this.command = command;
            this.distance = distance;
        }
    }
}
