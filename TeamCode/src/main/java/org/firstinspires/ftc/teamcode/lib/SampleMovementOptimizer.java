package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;

public class SampleMovementOptimizer {

    /**
     * Finds the closest point to the sample while staying a safe distance away.
     * @return The optimized Pose2d.
     */
    public static Pose2d getClosestPoint(Pose2d samplePos, double intermediateX, double intermediateYOffset, double sampleRadius) {
        Vector2d sampleVec = new Vector2d(samplePos.getX(), samplePos.getY());
        Vector2d intermediateVec = new Vector2d(intermediateX, Math.max(0.0, sampleVec.getY() + intermediateYOffset));
        Vector2d edge = sampleVec.minus(intermediateVec);
        double distance = edge.magnitude() - sampleRadius;

        if (distance < 0) {
            return getClosestPoint(samplePos, intermediateX, intermediateYOffset + 10, sampleRadius);
        }

        Vector2d point = intermediateVec.plus(edge.normalize().scale(distance));
        double angle = point.minus(intermediateVec).angle();
        return new Pose2d(point.getX(), point.getY(), new Rotation2d(angle));
    }

    /**
     * Computes an intermediate waypoint that helps guide the robot toward the sample efficiently.
     * @return The optimized Pose2d for the waypoint.
     */
    public static Pose2d getIntermediatePoint(Pose2d samplePos, double intermediateX, double intermediateYOffset, double sampleRadius) {
        Pose2d closestPoint = getClosestPoint(samplePos, intermediateX, intermediateYOffset, sampleRadius);
        Vector2d closestVec = new Vector2d(closestPoint.getX(), closestPoint.getY());
        Vector2d scoreVec = new Vector2d(AutoConstants.scorePos.getX(), AutoConstants.scorePos.getY());
        Vector2d intermediateVec = new Vector2d(intermediateX, Math.max(0.0, samplePos.getY() + intermediateYOffset));

        Vector2d pathVec = intermediateVec.minus(scoreVec);

        return new Pose2d(intermediateVec.getX(), intermediateVec.getY(), new Rotation2d(pathVec.angle()));
    }
}