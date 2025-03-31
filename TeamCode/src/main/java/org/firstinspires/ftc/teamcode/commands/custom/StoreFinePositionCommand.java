package org.firstinspires.ftc.teamcode.commands.custom;

import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cx;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cy;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.focalL;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.lib.SlidePnP;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.Objects;

public class StoreFinePositionCommand extends CommandBase {

    private boolean done = false;
    private VisionSubsystem vision;
    private SamplePoseStorage storage;
    private PinpointSubsystem pinpoint;
    private PivotSubsystem pivot;
    private ExtensionSubsystem extend;
    private TurretSubsystem turret;


    public StoreFinePositionCommand(VisionSubsystem vision, SamplePoseStorage storage, PinpointSubsystem pinpoint, PivotSubsystem pivot, ExtensionSubsystem extend, TurretSubsystem turret) {
        this.vision = vision;
        this.pinpoint = pinpoint;
        this.storage = storage;
        this.pivot = pivot;
        this.extend = extend;
        this.turret = turret;
        addRequirements(turret, vision);
    }

    @Override
    public void execute() {
        Pose2d samplePos = vision.getSamplePose();

        if (Objects.isNull(samplePos)) {
            Log.w("FineAlign", "No sample detected. Skipping frame.");
            return;
        }

        SlidePnP pnp = new SlidePnP(cx, cy, focalL);
        SlidePnP.rz = Math.sin(Math.toRadians(pivot.getCurrentPosition())) * extend.getCurrentInches() + IVKConstants.pivotPointHeight;
        Log.v("FineAlign", "RZ: " + SlidePnP.rz);
        SlidePnP.rp = Math.toRadians(-90 + pivot.getCurrentPosition());
        Translation2d fieldSamp = pnp.getFieldCoordinates((int) samplePos.getY(), (int) -samplePos.getX(), pinpoint.getPose());
        Log.i("FineAlign", "Sample X: " + fieldSamp.getX());
        Log.i("FineAlign", "Sample Y: " + fieldSamp.getY());

        storage.setFinePosition(new Pose2d(fieldSamp.getX(), fieldSamp.getY(), pinpoint.getPose().getRotation()));
        turret.rotateTo(samplePos.getRotation().getDegrees());
        done = true;
        vision.setCam(true);

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        // if we end early, set the position to current position to not disrupt stuff
        if (interrupted) {
            storage.setFinePosition(pinpoint.getPose());
        }
    }

}
