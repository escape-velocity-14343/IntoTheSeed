package org.firstinspires.ftc.teamcode.lib;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

@Config
public class RobotPnP {
    double cx,cy,fl;
    public static double rx = 4.5;
    public static double ry = 6;
    public static double rz = 11.625;
    public static double rp = Math.toRadians(-20); //robot xyz, robot pitch
    Pose2d fieldPos;

    public RobotPnP(double cx, double cy, double fl) {
        this.cx = cx;
        this.cy = cy;
        this.fl = fl;
    }
    public Translation2d getRobotCentricTranslation(int px, int py) {
        double gt = Math.atan((py-cy)/fl);
        Log.v("camera centric angle", gt + "");
        double theta = rp-gt;
        Log.v("ground centric angle", theta + "");
        double gd = rz / Math.tan(theta);
        Log.v("ground distance", gd + "");
        double dist = Math.hypot(gd, rz);
        Log.v("sample distance", dist + "");
        double pd = Math.cos(gt)*dist;
        Log.v("plane distance", pd + "");
        double gx = (px-cx)*pd/fl;
        Log.v("ground horizontal distance", gx + "");
        // gd is backwards for some reason
        return new Translation2d(-gd + rx, ry-gx);
    }

    public Translation2d getFieldCoordinates(int px, int py, Pose2d robotPose) {

        Translation2d rctrans = getRobotCentricTranslation(px, py);

        return robotPose.getTranslation().plus(rctrans.rotateBy(robotPose.getRotation()));

    }


}
