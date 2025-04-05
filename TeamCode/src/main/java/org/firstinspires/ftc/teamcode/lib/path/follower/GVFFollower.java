package org.firstinspires.ftc.teamcode.lib.path.follower;

import static org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand.translationkP;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

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
    private ArrayList<Vector2d> trail = new ArrayList<>();

    private boolean useSquid = false;

    /**
     * Higher values means the bot sticks closer to the path.
     */
    public static double correctionP = 0.1;

    /**
     * Higher values means the bot follows curvature more. This value is able to make
     * the bot follow curvature too much.
     */
    public static double curvatureP = 0;

    /**
     * Increase if the bot isn't going forwards at full speed.
     */
    public static double forwardMult = 1.0;

    /**
     * Number of inches before the bot transitions to the next spline.
     */
    public static double splineTransitionInches = 2.0;

    /**
     * Tells GVF when to switch to SquID 2.0.
     */
    public static double pathEndUsageInches = 15.0;

    public static double headingLookaheadScalar = 15.0 / 40.0;

    private Pose2d lastPose;

    public GVFFollower() {

    }

    /**
     * Returns a Pose2d that can be fed directly into the drivebase as (x, y, heading).
     * Does not do end of path logic!!!
     */
    public Pose2d update(Pose2d robotPose, Pose2d robotVelocity) {

        Spline currentSpline = splines.get(currentSplineIndex);

        Vector2d pos = new Vector2d(robotPose.getX(), robotPose.getY());

        lastPose = robotPose;

        double closestT = Spline.projectPos(pos, currentSpline);

        // increment to next spline as much as needed
        while (closestT >= currentSpline.getLength() - splineTransitionInches && currentSplineIndex != splines.size() - 1) {
            currentSplineIndex += 1;

            currentSpline = splines.get(currentSplineIndex);
            closestT = Spline.projectPos(pos, currentSpline);
        }

        Log.v("GVFf", "closest t: " + closestT);

        // get forward vector
        Vector2d forwardVec = currentSpline.getVelocity(closestT).normalize().scale(forwardMult);
        Log.v("GVFf", "vec1: " + forwardVec.getX() + ", " + forwardVec.getY());

        // get perp vector
        Vector2d perpVecNoScale = currentSpline.getValue(closestT).minus(pos);
        Vector2d perpendicularVec = perpVecNoScale.scale(correctionP);
        Log.v("GVFf", "vec2: " + perpendicularVec.getX() + ", " + perpendicularVec.getY());

        // get curvature vector
        // TODO: check the sign on this
        Vector2d curvatureVec = new Vector2d();
        if (perpVecNoScale.magnitude() > 0.001) {
            curvatureVec = perpVecNoScale.normalize().scale(Spline.getCurvature(closestT, currentSpline) * curvatureP);
            Log.v("GVFf", "vec3: " + curvatureVec.getX() + ", " + curvatureVec.getY());
        }
        // add!
        Vector2d movementVec = forwardVec.plus(perpendicularVec).plus(curvatureVec);
        Log.v("GVFf", "vec4: " + movementVec.getX() + ", " + movementVec.getY());

        // point directly towards end if we are over the path, ignore all else
        if (closestT >= currentSpline.getLength() - 0.001 && currentSplineIndex == splines.size() - 1) {
            movementVec = currentSpline.getValue(currentSpline.getLength()).minus(pos).normalize().scale(forwardMult);
            Log.v("GVFf", "thing was triggeredf");
        }

        // if we are close to the end of the path, use squid 2.0 instead
        Spline lastSpline = splines.get(splines.size() - 1);
        Vector2d endpoint = lastSpline.getValue(lastSpline.getLength());

        // angle logic!
        // look ahead a little bit to get the angle
        double angleT = closestT + headingLookaheadScalar * robotVelocity.getTranslation().getNorm();

        // while angle t overflows, decrement by spline length and increment splines
        Spline currAngleSpline = currentSpline;
        int currAngleIndex = currentSplineIndex;
        while (angleT > currAngleSpline.getLength() && currAngleIndex != splines.size() - 1) {
            angleT -= currAngleSpline.getLength();
            currAngleIndex++;
            currAngleSpline = splines.get(currAngleIndex);
        }

        double targetAngle = Math.toDegrees(currAngleSpline.getVelocity(angleT).angle());

        if (pos.minus(endpoint).magnitude() < pathEndUsageInches) {
            useSquid = true;
        }

        if (useSquid) {
            drivetrainSquIDController.setPID(translationkP);

            Pose2d xyMove =
                    drivetrainSquIDController.calculate(
                            new Pose2d(endpoint.getX(), endpoint.getY(), new Rotation2d()),
                            robotPose,
                            robotVelocity);

            movementVec = new Vector2d(xyMove.getX(), xyMove.getY());
            targetAngle = Math.toDegrees(lastSpline.getVelocity(lastSpline.getLength()).angle());
        }



        // squid
        double rot = Util.signedSqrt(-Util.getAngularDifference(targetAngle, robotPose.getRotation().getDegrees()) * DefaultGoToPointCommand.headingkP);
        Log.v("GVFf", "attempted movement: " + movementVec.getX() + ", " + movementVec.getY() + ", " + rot);

        //FtcDashboard.getInstance().sendTelemetryPacket(getSplineDrawPacket(100));

        TelemetryPacket packet = new TelemetryPacket();
        getRobotDrawPacket(packet);
        getSplineDrawPacket(packet, 100);

        double[] trailX = new double[trail.size()];
        double[] trailY = new double[trail.size()];

        for (int i = 0; i < trail.size(); i++) {
            trailX[i] = trail.get(i).getX();
            trailY[i] = trail.get(i).getY();
        }

        packet.fieldOverlay()
                .setStroke("#AAAAAA88")
                .setStrokeWidth(1)
                .strokePolyline(
                        trailX,
                        trailY
                );
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        trail.add(pos);

        return new Pose2d(movementVec.getX(), movementVec.getY(), Rotation2d.fromDegrees(rot));
    }

    public Pose2d getEndpoint() {
        Spline lastSpline = splines.get(splines.size() - 1);
        Vector2d endpoint = lastSpline.getValue(lastSpline.getLength());
        return new Pose2d(endpoint.getX(), endpoint.getY(), new Rotation2d(lastSpline.getVelocity(lastSpline.getLength()).angle()));
    }

    public void setSplines(Spline... splines) {
        this.splines = new ArrayList<>(Arrays.asList(splines));
        this.useSquid = false;
        for (Spline spline : this.splines) {
            spline.initArclen();
        }
        currentSplineIndex = 0;
    }

    public void getSplineDrawPacket(TelemetryPacket packet, int precision) {
        double[] pointsX = new double[precision];
        double[] pointsY = new double[precision];

        for (int i = 0; i < precision; i++) {
            double t = i / (double) precision * splines.size();
            int splineIndex = (int) t;
            double splineT = t % splines.size();
            Vector2d p = splines.get(splineIndex).getValue(splineT * splines.get(splineIndex).getLength());
            pointsX[i] = p.getX();
            pointsY[i] = p.getY();
        }

        packet.fieldOverlay()
                .setStroke("red")
                .setStrokeWidth(1)
                .strokePolyline(pointsX, pointsY);

    }

    public void getRobotDrawPacket(TelemetryPacket packet) {
        double rot = lastPose.getRotation().getRadians();
        Vector2d headingPos = new Vector2d(Math.cos(rot), Math.sin(rot)).scale(3.5);

        packet.fieldOverlay()
                .setStroke("blue")
                .strokeCircle(lastPose.getX(), lastPose.getY(), 7)
                .strokeLine(lastPose.getX(), lastPose.getY(),
                        lastPose.getX() + headingPos.getX(), lastPose.getY() + headingPos.getY()
                );

    }
}
