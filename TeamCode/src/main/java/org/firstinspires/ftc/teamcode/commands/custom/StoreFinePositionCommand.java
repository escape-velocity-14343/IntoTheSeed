package org.firstinspires.ftc.teamcode.commands.custom;

import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cx;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cy;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.focalL;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
import java.util.concurrent.Delayed;

public class StoreFinePositionCommand extends CommandBase {

    private boolean done = false;
    private VisionSubsystem vision;
    private SamplePoseStorage storage;
    private PinpointSubsystem pinpoint;
    private PivotSubsystem pivot;
    private ExtensionSubsystem extend;
    private TurretSubsystem turret;

    private boolean drawn = false;

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
    public void initialize() {
        storage.setFinePosition(pinpoint.getPose());
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
        Log.v("FineAlign", "RP (deg): " + Math.toDegrees(SlidePnP.rp));
        Translation2d rcSamp = pnp.getRobotCentricTranslation((int) samplePos.getX(), (int) samplePos.getY());
        Translation2d fieldSamp = pnp.getFieldCoordinates((int) samplePos.getX(), (int) samplePos.getY(), pinpoint.getPose());


        // now we know where the sample is relative to the camera
        // we now have to figure out where the camera is
//        double forwardExtension = Math.cos(Math.toRadians(pivot.getCurrentPosition())) * extend.getCurrentInches() + (IVKConstants.lengthOfBot / 2);
        double forwardExtension = Math.cos(Math.toRadians(pivot.getCurrentPosition())) * extend.getCurrentInches() + IVKConstants.slideLength * Math.cos(Math.toRadians(pivot.getCurrentPosition())) - IVKConstants.slideRotationOffset;

        fieldSamp.plus(new Translation2d(forwardExtension, 0).rotateBy(pinpoint.getPose().getRotation()));

        Log.i("FineAlign", "Sample X: " + fieldSamp.getX());
        Log.i("FineAlign", "Sample Y: " + fieldSamp.getY());

        // if too close to edge, offset
        /*if (!(fieldSamp.getY() < 11.5 && fieldSamp.getY() > -11.5
                && fieldSamp.getX() < 20.0 && fieldSamp.getX() > -20.0)) {
            Translation2d fieldSampNew = new Translation2d(Range.clip(fieldSamp.getX(), -20.0, 20.0), Range.clip(fieldSamp.getY(), -11.5, 11.5));
            Translation2d delta = fieldSampNew.minus(fieldSamp);
            Log.i("FineAlign", "Delta X: " + delta.getX());
            Log.i("FineAlign", "Delta Y: " + delta.getY());
            rcSamp = rcSamp.plus(delta.rotateBy(pinpoint.getPose().getRotation().times(-1)));
        }*/

        // sample position relative to robot
        Vector2d rcSampleVector = new Vector2d(forwardExtension, 0).plus(new Vector2d(rcSamp.getX(), rcSamp.getY()));
        Log.i("FineAlign", "rc sample x: " + rcSampleVector.getX());
        Log.i("FineAlign", "rc sample y: " + rcSampleVector.getY());

        // compute angle to turn
        double deltaAngle = Math.toDegrees(rcSampleVector.angle());
        Log.i("FineAlign", "delta angle: " + deltaAngle);

        // compute extension to add
        double deltaExtension = rcSampleVector.magnitude() - forwardExtension - IVKConstants.slideBackOffset;
        Log.i("FineAlign", "delta extension: " + deltaExtension);

        double newAngle = pinpoint.getPose().getRotation().getDegrees() + deltaAngle;
        double newExtension = extend.getCurrentInches() + deltaExtension;

        storage.setFinePosition(new Pose2d(pinpoint.getPose().getX(), pinpoint.getPose().getY(),
                new Rotation2d(Math.toRadians(newAngle))));
        storage.setNewExtension(newExtension);
        turret.rotateTo(AngleUnit.normalizeDegrees(samplePos.getRotation().getDegrees() - deltaAngle));
        done = true;
        //vision.setCam(true);
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
            storage.setNewExtension(extend.getCurrentInches());
        }
    }
}
