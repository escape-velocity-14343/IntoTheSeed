package org.firstinspires.ftc.teamcode.lib.path.spline;

import com.arcrobotics.ftclib.geometry.Vector2d;

import java.util.ArrayList;

public class Lerp extends SplineBase {
    // i was insane when i wrote this why on earth are arraylists used
    // anyways it works don't question it
    public ArrayList<Vector2d> controlpoints;

    public Lerp(ArrayList<Vector2d> controls) {
        assert controls.size() == 2;
        controlpoints = controls;
        setLength(1.0);
    }

    public Lerp(Vector2d first, Vector2d second) {
        this(new ArrayList<Vector2d>(){{this.add(first);this.add(second);}});
    }

    public Vector2d getValue(double t) {
        t = constrainNormalized(t);
        return controlpoints.get(0).scale(1-t).plus(controlpoints.get(1).scale(t));
    }

    public Vector2d getVelocity(double t) {
        return controlpoints.get(0).scale(-1).plus(controlpoints.get(1));
    }

    public Vector2d getAcceleration(double t) {
        return new Vector2d();
    }
}