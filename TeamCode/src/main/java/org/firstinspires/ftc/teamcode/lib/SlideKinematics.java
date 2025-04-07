package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.constants.IVKConstants;

public class SlideKinematics {

    /**
     * Returns the position of the end effector, relative to the robot (y is height).
     */
    public static Vector2d getEndEffectorPosition(double angle, double magnitude) {

        Vector2d slideEnd = new Vector2d(1, 0).rotateBy(angle).scale(magnitude);
        slideEnd = slideEnd.plus(new Vector2d(0, IVKConstants.pivotPointHeight));

        return slideEnd;

    }

}
