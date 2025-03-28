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

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

public class CoarseAlignCommand extends CommandBase {

    DefaultGoToPointCommand gtpc;
    PinpointSubsystem pinpoint;
    VisionSubsystem vision;

    enum DetectionState {
        SCANNING,
        MOVING
    }

    private DetectionState state = DetectionState.SCANNING;


    public CoarseAlignCommand(DefaultGoToPointCommand gtpc, VisionSubsystem vision, PinpointSubsystem pinpoint) {
        this.gtpc = gtpc;
        this.vision = vision;
        this.pinpoint = pinpoint;
    }

    @Override
    public void initialize() {
        // it's set here but should really be set much earlier
        vision.setCam(true);
    }

    @Override
    public void execute() {
        switch (state) {
            case SCANNING:
                ArrayList<Vector2d> samplePoses = vision.getSamplePoses();

                // iterate through all potential samples, pick only legal ones
                for (Vector2d samp : samplePoses) {
                    RobotPnP pnp = new RobotPnP(cx, cy, focalL);
                    Translation2d fieldSamp = pnp.getFieldCoordinates((int) samp.getX(), (int) samp.getY(), pinpoint.getPose());
                    Log.i("CoarseAlign", "Sample X: " + fieldSamp.getX());
                    Log.i("CoarseAlign", "Sample Y: " + fieldSamp.getY());

                    if (-15 < fieldSamp.getX() && fieldSamp.getX() < 15
                    && -13 < fieldSamp.getY() && fieldSamp.getY() < 13) {
                        gtpc.setTarget(new Pose2d(fieldSamp.getX(), fieldSamp.getY() + 25, Rotation2d.fromDegrees(-90)));
                        vision.setCam(false);
                        state = DetectionState.MOVING;
                        break;
                    }
                }

                Log.w("CoarseAlign", "No sample detected. Skipping frame.");
                break;

            case MOVING:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        vision.setCam(false);
    }

    @Override
    public boolean isFinished() {
        return gtpc.isDone() && state == DetectionState.MOVING;
    }
}
