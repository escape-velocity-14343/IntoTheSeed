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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.lib.AnalogEncoder;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.lib.RobotSlidePnP;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.lib.SlideKinematics;
import org.firstinspires.ftc.teamcode.lib.SlidePnP;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

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

    public static Pose2d lastCameraPos = new Pose2d(0,0, new Rotation2d(-1));

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

        RobotSlidePnP pnp = new RobotSlidePnP(cy, cx, focalL);
        if (lastCameraPos.getRotation().getRadians()==-1) {
            lastCameraPos = SlideKinematics.getRCCameraPos(Rotation2d.fromDegrees(pivot.getCurrentPosition()), extend.getCurrentInches());
        }
        lastCameraPos = SlideKinematics.getRCCameraPos(Rotation2d.fromDegrees(pivot.getCurrentPosition()), extend.getCurrentInches());

        RobotSlidePnP.rx = lastCameraPos.getX();
        RobotSlidePnP.rz = lastCameraPos.getY();
        RobotSlidePnP.rp = Math.toRadians(-90 + pivot.getCurrentPosition());



        Log.v("FineAlign", "RX: " + RobotSlidePnP.rx);
        Log.v("FineAlign", "RZ: " + RobotSlidePnP.rz);
        Log.v("FineAlign", "RP (deg): " + Math.toDegrees(RobotSlidePnP.rp));

        Translation2d rcSamp = pnp.getRobotCentricTranslation((int) (samplePos.getY()), (int) (320-samplePos.getX()));
        Translation2d fcSamp = pnp.getFieldCoordinates((int) (samplePos.getY()), (int) (320-samplePos.getX()), pinpoint.getPose());
        Log.i("FineAlign", "RC Sample X: " + rcSamp.getX());
        Log.i("FineAlign", "RC Sample Y: " + rcSamp.getY());

        Log.i("FineAlign", "FC Sample X: " + fcSamp.getX());
        Log.i("FineAlign", "FC Sample Y: " + fcSamp.getY());
        fcSamp = new Translation2d(Util.clamp(0, -23, fcSamp.getX()), Util.clamp(11, -11, fcSamp.getY()));

        // sample position relative to robot
        Vector2d rcSampleVector = new Vector2d(rcSamp.getX(), rcSamp.getY());

        // compute angle to turn
        double deltaAngle = Math.toDegrees(rcSampleVector.angle());
        Log.i("FineAlign", "delta angle: " + deltaAngle);

        double newAngle = pinpoint.getPose().getRotation().getDegrees() + deltaAngle;
        double newExtension = SlideKinematics.getIVKClawPos(new Translation2d(rcSamp.getX(), IVKConstants.clawIntakeIVKHeight)).getX() * IVKConstants.extensionScalar;

        storage.setFinePosition(new Pose2d(pinpoint.getPose().getX(), pinpoint.getPose().getY(),
                new Rotation2d(Math.toRadians(newAngle))));
        storage.setNewExtension(Range.clip(rcSamp.getX(), 4, 34));
        turret.rotateTo(AnalogEncoder.normalizeDegrees(samplePos.getRotation().getDegrees() - deltaAngle));
        done = true;

    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
