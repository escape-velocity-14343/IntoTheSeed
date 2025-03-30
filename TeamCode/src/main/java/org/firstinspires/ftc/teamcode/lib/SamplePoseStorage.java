package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class SamplePoseStorage {

    public Pose2d coarsePosition = new Pose2d();
    public Pose2d finePosition = new Pose2d();


    public SamplePoseStorage() {

    }


    public Pose2d getCoarsePosition() {
        return coarsePosition;
    }

    public Pose2d getCoarsePositionOffset() {
        return new Pose2d(coarsePosition.getX(), coarsePosition.getY() + 33, Rotation2d.fromDegrees(-90));
    }

    public void setCoarsePosition(Pose2d coarsePosition) {
        this.coarsePosition = coarsePosition;
    }

    public Pose2d getFinePosition() {
        return finePosition;
    }

    public void setFinePosition(Pose2d finePosition) {
        this.finePosition = finePosition;
    }
}
