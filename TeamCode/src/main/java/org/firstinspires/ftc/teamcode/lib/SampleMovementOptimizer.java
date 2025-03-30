package org.firstinspires.ftc.teamcode.lib;


import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class SampleMovementOptimizer {

    /**
     * Given where the sample is and where the robot is, find the pose that minimizes the distance between sample and point.
     * @return
     */
    public static Pose2d getClosestPoint(Pose2d samplePos, double intermediateX, double intermediateYOffset, double sampleRadius) {
        Vector2d sampleVec = new Vector2d(samplePos.getX(), samplePos.getY());
        Vector2d intermediateVec = new Vector2d(intermediateX, Math.max(38.0, sampleVec.getY() + intermediateYOffset));
        Vector2d edge = sampleVec.minus(intermediateVec);
        double distance = edge.magnitude() - sampleRadius;
        Vector2d point = intermediateVec.plus(edge.normalize().scale(distance));

        double angle = point.minus(intermediateVec).angle();

        return new Pose2d(point.getX(), point.getY(), new Rotation2d(angle));

    }

    public static Pose2d getIntermediatePoint(Pose2d samplePos, double intermediateX, double intermediateYOffset, double sampleRadius) {
        Pose2d end = SampleMovementOptimizer.getClosestPoint(samplePos, intermediateX, intermediateYOffset, sampleRadius);
        return new Pose2d(intermediateX, Math.max(38.0, samplePos.getY() + intermediateYOffset), end.getRotation());
    }

}
