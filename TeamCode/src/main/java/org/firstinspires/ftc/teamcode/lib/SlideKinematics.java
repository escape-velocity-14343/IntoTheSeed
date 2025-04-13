package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.constants.IVKConstants;


public class SlideKinematics {

    /**
     * Returns the position of the end effector, relative to the robot (y is height).
     */
    /*public static Vector2d getEndEffectorPosition(double angle, double magnitude) {

        Vector2d slideEnd = new Vector2d(1, 0).rotateBy(angle).scale(magnitude);
        slideEnd = slideEnd.plus(new Vector2d(0, IVKConstants.pivotPointHeight));

        return slideEnd;

    }*/
    /**
     * @return x is forward, y is height
     * */
    public static Pose2d getRCCameraPos(Rotation2d angle, double extension) {
        double slideRelativeExtension = extension + IVKConstants.cameraOffsetForward;
        double forward = IVKConstants.pivotPointForwardCam + angle.getCos() * slideRelativeExtension - angle.getSin() * IVKConstants.cameraOffsetUp;
        double height = IVKConstants.pivotPointHeightCam + angle.getSin() * slideRelativeExtension + angle.getCos() * IVKConstants.cameraOffsetUp;
        return new Pose2d(forward, height, angle);
    }


}
