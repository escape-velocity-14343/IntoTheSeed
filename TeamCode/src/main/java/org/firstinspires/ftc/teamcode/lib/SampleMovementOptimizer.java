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
        Vector2d intermediateVec = new Vector2d(intermediateX, Math.max(38.0, sampleVec.getY() + intermediateYOffset));

        // Compute direction from intermediate to sample
        Vector2d edge = sampleVec.minus(intermediateVec);
        if (edge.magnitude() < sampleRadius) {
            // If already inside the sample's safe zone, return the original intermediate position
            return new Pose2d(intermediateVec.getX(), intermediateVec.getY(), new Rotation2d(0));
        }

        Vector2d safeOffset = edge.normalize().times(edge.magnitude() - sampleRadius);
        Vector2d closestPoint = intermediateVec.plus(safeOffset);
        double angle = Math.atan2(edge.getY(), edge.getX()); // Corrected rotation angle computation

        return new Pose2d(closestPoint.getX(), closestPoint.getY(), new Rotation2d(angle));
    }

    /**
     * Computes an intermediate waypoint that helps guide the robot toward the sample efficiently.
     * @return The optimized Pose2d for the waypoint.
     */
    public static Pose2d getIntermediatePoint(Pose2d samplePos, double intermediateX, double intermediateYOffset, double sampleRadius) {
        Pose2d closestPoint = getClosestPoint(samplePos, intermediateX, intermediateYOffset, sampleRadius);
        Vector2d closestVec = new Vector2d(closestPoint.getX(), closestPoint.getY());
        Vector2d scoreVec = new Vector2d(AutoConstants.scorePos.getX(), AutoConstants.scorePos.getY());

        // Compute the vector from scoring position to the closest point
        Vector2d pathVec = closestVec.minus(scoreVec).normalize().times(20.0).plus(scoreVec);

        return new Pose2d(pathVec.getX(), pathVec.getY(), new Rotation2d(Math.atan2(pathVec.getY() - scoreVec.getY(), pathVec.getX() - scoreVec.getX())));
    }
}