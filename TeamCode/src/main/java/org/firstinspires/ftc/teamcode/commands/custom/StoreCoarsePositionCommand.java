package org.firstinspires.ftc.teamcode.commands.custom;

import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cx;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.cy;
import static org.firstinspires.ftc.teamcode.constants.VisionConstants.focalL;

import android.util.Log;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.lib.RobotPnP;
import org.firstinspires.ftc.teamcode.lib.SamplePoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

import java.util.ArrayList;

public class StoreCoarsePositionCommand extends CommandBase {

    private boolean done = false;
    private VisionSubsystem vision;
    private SamplePoseStorage storage;
    private PinpointSubsystem pinpoint;


    public StoreCoarsePositionCommand(VisionSubsystem vision, SamplePoseStorage storage, PinpointSubsystem pinpoint) {
        this.vision = vision;
        this.pinpoint = pinpoint;
        this.storage = storage;
    }

    @Override
    public void execute() {
        ArrayList<Vector2d> samplePoses = vision.getSamplePoses();

        // iterate through all potential samples, pick only legal ones
        for (Vector2d samp : samplePoses) {
            RobotPnP pnp = new RobotPnP(cx, cy, focalL);
            Translation2d fieldSamp = pnp.getFieldCoordinates((int) samp.getX(), (int) samp.getY(), pinpoint.getPose());
            Log.i("CoarseAlign", "Sample X: " + fieldSamp.getX());
            Log.i("CoarseAlign", "Sample Y: " + fieldSamp.getY());

            if (-10 < fieldSamp.getX() && fieldSamp.getX() < 20
                    && -5 < fieldSamp.getY() && fieldSamp.getY() < 13) {
                storage.setCoarsePosition(new Pose2d(fieldSamp.getX(), fieldSamp.getY(), new Rotation2d()));
                done = true;
                vision.setCam(false);
                return;
            }
        }

        Log.w("CoarseAlign", "No sample detected. Skipping frame.");

    }

    @Override
    public boolean isFinished() {
        return done;
    }

}
