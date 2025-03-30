package org.firstinspires.ftc.teamcode.lib.path.spline;

import com.arcrobotics.ftclib.geometry.Vector2d;

/**
 * bad code structure go brrr
 * Implements length and arc length for splines by default.
 */
public abstract class SplineBase implements Spline {

    protected double length;

    public SplineBase() {
        this.length = 1.0;
    }

    @Override
    public double getLength() {
        return length;
    }

    @Override
    public void setLength(double length) {
        this.length = length;
    }

    /**
     * Brute-force linear approximation of arclength.
     * @param points Increase for higher accuracy. Generates more subdivisions during approximation.
     */
    protected double getArclength(int points) {

        Vector2d lastPoint = getValue(0);
        Vector2d currentPoint = getValue(0);

        double arclength = 0;

        for (int i = 1; i <= points; i++) {
            double t = ((double) i) / ((double) points); // generate t from [0-1]
            t *= getLength(); // scale

            currentPoint = getValue(t); // grab current point

            // add arclength to sum and continue
            arclength += currentPoint.minus(lastPoint).magnitude();

            lastPoint = currentPoint;
        }

        return arclength;

    }

    /**
     * Brute-force linear approximation of arclength. Override if higher accuracy is needed,
     * using the <code>getArclength(int)</code> method.
     */
    @Override
    public double getArclength() {
        return getArclength(100);
    }

}
