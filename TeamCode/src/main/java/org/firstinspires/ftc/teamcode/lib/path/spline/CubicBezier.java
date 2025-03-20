package org.firstinspires.ftc.teamcode.lib.path.spline;

import com.arcrobotics.ftclib.geometry.Vector2d;

import java.util.ArrayList;

public class CubicBezier extends SplineBase {
    public ArrayList<Vector2d> controlpoints;

    public CubicBezier(ArrayList<Vector2d> controls) {
        assert controls.size() == 4;
        controlpoints = controls;
        setLength(1.0);
    }
    public Vector2d getValue(double t) {
        t = constrainNormalized(t);
        return controlpoints.get(0).scale(Math.pow(1-t, 3))
                .plus(controlpoints.get(1).scale(3*Math.pow(1-t, 2)*t))
                .plus(controlpoints.get(2).scale(3*Math.pow(t, 2)*(1-t)))
                .plus(controlpoints.get(3).scale(Math.pow(t, 3)));
    }

    public Vector2d getVelocity(double t) {
        t = constrainNormalized(t);
        return controlpoints.get(1).minus(controlpoints.get(0))
                        .scale(3*Math.pow(1-t, 2))

                .plus(controlpoints.get(2).minus(controlpoints.get(1))
                        .scale(6*(1-t)))

                .plus(controlpoints.get(3).minus(controlpoints.get(2))
                        .scale(3*Math.pow(t, 2)));

    }

    public Vector2d getAcceleration(double t) {
        t = constrain(t);
        return controlpoints.get(2)
                .plus(controlpoints.get(1).scale(-2))
                .plus(controlpoints.get(0))
                .scale(6*(1-t))

                        .plus(
                                controlpoints.get(3)
                                        .plus(controlpoints.get(2).scale(-2))
                                        .plus(controlpoints.get(1))
                                        .scale(6*(t)));

    }
}