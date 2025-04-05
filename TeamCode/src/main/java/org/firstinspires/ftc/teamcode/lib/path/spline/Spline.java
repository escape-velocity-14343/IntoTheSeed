package org.firstinspires.ftc.teamcode.lib.path.spline;

import com.arcrobotics.ftclib.geometry.Vector2d;

import java.util.Objects;

/**
 * Standard spline interface - defines a smoothly continuous parametric function R -> R^2.
 */
public interface Spline {

    /**
     * @return t in [0, maxT].
     */
    default double constrain(double t) {
        return Math.min(Math.max(0, t), getLength());
    }

    /**
     * @return t in [0, 1].
     */
    default double constrainNormalized(double t) {
        return constrain(t) / getLength();
    }

    Vector2d getValue(double t);

    Vector2d getVelocity(double t);

    Vector2d getAcceleration(double t);

    /**
     * This will shift the t range to arclength parameterization ([0, arclen]).
     */
    default void initArclen() {
        setLength(getArclength());
    };

    /**
     * @return The max t value, i.e. if the spline is a parametric function defined from [0, maxT], this function returns maxT.
     */
    double getLength();

    void setLength(double length);

    /**
     * @return The arclength.
     */
    double getArclength();

    /**
     * Utility function to find the closest point on a spline, given a position <code>pos</code>.
     * @param tolerance Decrease to increase accuracy at the cost of more iterations.
     *                  Wondering what unit this is in? hell if i know lol it's weird
     *                  (ask if you want to understand but otherwise just decrease the number)
     * @param initialPoints Increase to increase accuracy.
     *                      Generally, use 25 or call the function without this argument.
     *                      Ask Michael if this is causing issues.
     */
    static double projectPos(Vector2d pos, Spline spline, double tolerance, int initialPoints) {

        // initialize lists
        double[] currVals = new double[initialPoints];
        Vector2d[] currVecs = new Vector2d[initialPoints];

        // minDist is our minimum distance to an initial point, and ind is the index of that initial point
        double minDist = Double.POSITIVE_INFINITY;
        int ind = 0;
        double dist = 0;

        for (int i = 0; i < initialPoints; i++) {

            // each value is spaced evenly over the length of the curve
            currVals[i] = (i/((double) initialPoints-1))*spline.getLength();
            currVecs[i] = spline.getValue(currVals[i]);

            // find the distance from our position to each initial point and take the minimum distance
            dist = currVecs[i].minus(pos).magnitude();
            if (dist < minDist) {
                ind = i;
                minDist = dist;
            }
        }

        // curr represents our current optimal t value
        double curr = currVals[ind];

        // newtonian iteration black magic (thanks eeshwar)
        /* ok actually it makes sense lol, basically project the derivative of the spline
            at the current t value against the vector from the robot pos to the current spline
            pos, and that vector's length should be minimized,
         */
        for(int i = 0; i < 250; i++){
            Vector2d p = spline.getValue(curr);
            Vector2d deriv = spline.getVelocity(curr);

            double ds = deriv.dot(pos.minus(p));

            ds = ds / deriv.magnitude();

            if(-tolerance <= ds && ds <= tolerance) {
                break;
            }

            curr += ds / spline.getLength();

            curr = spline.constrain(curr);
        }

        return spline.constrain(curr);
    }

    static double projectPos(Vector2d pos, Spline spline) {
        // test this tolerance
        return Spline.projectPos(pos, spline, 0.01, 25);
    }

    /**
     * curvature formula ripped from wikipedia :)
     */
    static double getCurvature(double t, Spline spline) {
        t = spline.constrain(t);
        Vector2d velo = spline.getVelocity(t);
        Vector2d accel = spline.getAcceleration(t);
        double xd1 = velo.getX();
        double yd1 = velo.getY();
        double xd2 = accel.getX();
        double yd2 = accel.getY();
        double curvature = Math.pow(xd1*xd1+yd1*yd1, 1.5)/(xd1*yd2-xd2*yd1);
        return Double.isNaN(curvature) || Double.isInfinite(curvature) ? 0.0 : curvature;
    }

}