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
        Log.i("FineAlign", "Sample X: " + fieldSamp.getX());
        Log.i("FineAlign", "Sample Y: " + fieldSamp.getY());

        // now we know where the sample is relative to the camera
        // we now have to figure out where the camera is
        double forwardExtension = Math.cos(Math.toRadians(pivot.getCurrentPosition())) * extend.getCurrentInches() + (IVKConstants.lengthOfBot / 2);

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
        if (!drawn){
            TelemetryPacket packet = new TelemetryPacket();
            drawSample(packet.fieldOverlay(), fieldSamp, samplePos);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            drawn = true;
        }
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

    public void drawSample(Canvas c, Translation2d fieldSamp, Pose2d samplePos) {
        //Use samplePos for rotation of gamepiece
        //Use fieldSamp for field relative coordinates to draw on
        final double SAMPLE_LENGTH = 3.5;
        final double SAMPLE_WIDTH = 1.5;

        Translation2d frontLeft = fieldSamp.plus(new Translation2d(SAMPLE_LENGTH/2, SAMPLE_WIDTH/2).rotateBy(samplePos.getRotation().plus(Rotation2d.fromDegrees(-90))));
        Translation2d frontRight = fieldSamp.plus(new Translation2d(SAMPLE_LENGTH/2, -SAMPLE_WIDTH/2).rotateBy(samplePos.getRotation().plus(Rotation2d.fromDegrees(-90))));
        Translation2d backRight = fieldSamp.plus(new Translation2d(-SAMPLE_LENGTH/2, -SAMPLE_WIDTH/2).rotateBy(samplePos.getRotation().plus(Rotation2d.fromDegrees(-90))));
        Translation2d backLeft = fieldSamp.plus(new Translation2d(-SAMPLE_LENGTH/2, SAMPLE_WIDTH/2).rotateBy(samplePos.getRotation().plus(Rotation2d.fromDegrees(-90))));

        double[] xPoints = {frontLeft.getX(), frontRight.getX(), backRight.getX(), backLeft.getX()};
        double[] yPoints = {frontLeft.getY(), frontRight.getY(), backRight.getY(), backLeft.getY()};


        c.setStrokeWidth(1);
        c.strokePolygon(xPoints, yPoints);
    }

}
