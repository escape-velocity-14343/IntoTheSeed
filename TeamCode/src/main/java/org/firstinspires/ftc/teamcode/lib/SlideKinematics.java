package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.constants.IVKConstants;


public class SlideKinematics {


    /**
     * @return x is forward, y is height
     * */
    public static Pose2d getRCCameraPos(Rotation2d angle, double extension) {
        double slideRelativeExtension = extension + IVKConstants.cameraOffsetForward;
        double forward = IVKConstants.pivotPointForwardCam + angle.getCos() * slideRelativeExtension - angle.getSin() * IVKConstants.cameraOffsetUp;
        double height = IVKConstants.pivotPointHeightCam + angle.getSin() * slideRelativeExtension + angle.getCos() * IVKConstants.cameraOffsetUp;
        return new Pose2d(forward, height, angle);
    }
    public static Pose2d getIVKClawPos(Translation2d target) {
        return new Pose2d();
    }


}
