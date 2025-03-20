package org.firstinspires.ftc.teamcode.lib.path.follower;

import static org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand.translationkP;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.lib.DrivetrainSquIDController;
import org.firstinspires.ftc.teamcode.lib.Util;
import org.firstinspires.ftc.teamcode.lib.path.spline.Spline;

import java.util.ArrayList;
import java.util.Arrays;

@Config
public class GVFFollower {
    private ArrayList<Spline> splines;
    private int currentSplineIndex = 0;
    private DrivetrainSquIDController drivetrainSquIDController = new DrivetrainSquIDController();
    public static double correctionP = 0.1;
    public static double curvatureP = 0.1;

    /**
     * Increase if the bot isn't going forwards at full speed.
     */
    public static double forwardMult = 5;
    public static double splineTransitionInches = 2.0;

    /**
     * Tells GVF when to switch to SquID 2.0.
     */
    public static double pathEndUsageInches = 15.0;


    /**
     * Returns a Pose2d that can be fed directly into the drivebase as (x, y, heading).
     * Does not do end of path logic!!!
     */
    public Pose2d update(Pose2d robotPose, Pose2d robotVelocity) {

        Spline currentSpline = splines.get(currentSplineIndex);

        Vector2d pos = new Vector2d(robotPose.getX(), robotPose.getY());

        double closestT = Spline.projectPos(pos, currentSpline);

        // increment to next spline as much as needed
        while (closestT >= currentSpline.getLength() - splineTransitionInches && currentSplineIndex != splines.size() - 1) {
            currentSplineIndex += 1;

            currentSpline = splines.get(currentSplineIndex);
            closestT = Spline.projectPos(pos, currentSpline);
        }

        // get forward vector
        Vector2d forwardVec = currentSpline.getVelocity(closestT).normalize().scale(forwardMult);

        // get perp vector
        Vector2d perpendicularVec = currentSpline.getValue(closestT).minus(pos).scale(correctionP);

        // get curvature vector
        // TODO: check the sign on this
        Vector2d curvatureVec = perpendicularVec.normalize().scale(Spline.getCurvature(closestT, currentSpline) * curvatureP);

        // add!
        Vector2d movementVec = forwardVec.plus(perpendicularVec).plus(curvatureVec);

        // point directly towards end if we are over the path, ignore all else
        if (closestT >= currentSpline.getLength() - 0.001 && currentSplineIndex == splines.size() - 1) {
            movementVec = currentSpline.getValue(currentSpline.getLength()).minus(pos).normalize().scale(forwardMult);
        }

        // if we are close to the end of the path, use squid 2.0 instead
        Spline lastSpline = splines.get(splines.size() - 1);
        Vector2d endpoint = lastSpline.getValue(lastSpline.getLength());

        if (pos.minus(endpoint).magnitude() < pathEndUsageInches) {

            drivetrainSquIDController.setPID(translationkP);

            Pose2d xyMove =
                    drivetrainSquIDController.calculate(
                            new Pose2d(endpoint.getX(), endpoint.getY(), new Rotation2d()),
                            robotPose,
                            robotVelocity);

            movementVec = new Vector2d(xyMove.getX(), xyMove.getY());
        }

        // angle logic!
        double targetAngle = AngleUnit.normalizeDegrees(forwardVec.angle());

        // squid
        double rot = Util.signedSqrt(Util.getAngularDifference(targetAngle, robotPose.getRotation().getDegrees()) * DefaultGoToPointCommand.headingkP);

        return new Pose2d(movementVec.getX(), movementVec.getY(), Rotation2d.fromDegrees(rot));

    }

    public void setSplines(Spline... splines) {
        this.splines = new ArrayList<>(Arrays.asList(splines));
        for (Spline spline : this.splines) {
            spline.initArclen();
        }
        currentSplineIndex = 0;
    }



}
