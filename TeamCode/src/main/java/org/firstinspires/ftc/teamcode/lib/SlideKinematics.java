package org.firstinspires.ftc.teamcode.lib;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.commands.custom.ClampExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.SlowExtendCommand;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.function.DoubleSupplier;


public class SlideKinematics {


    /**
     * @return x is forward, y is height
     * */
    public static Pose2d getRCCameraPos(Rotation2d angle, double extension) {
        double slideRelativeExtension = extension + IVKConstants.cameraOffsetForward;
        double forward = IVKConstants.pivotPointForwardOffset + angle.getCos() * slideRelativeExtension - angle.getSin() * IVKConstants.cameraOffsetUp;
        double height = IVKConstants.pivotPointHeightOffset + angle.getSin() * slideRelativeExtension + angle.getCos() * IVKConstants.cameraOffsetUp;
        return new Pose2d(forward, height, angle);
    }
    public static Pose2d getIVKClawPos(Translation2d target) {
        target = target.minus(new Translation2d(IVKConstants.pivotPointForwardOffset, IVKConstants.pivotPointHeightOffset));
        Log.v("IVK", "");
        double targetTheta = Math.atan2(target.getY(), target.getX());
        Log.v("IVK", "Target Theta: " + targetTheta);
        double targetDist = Math.hypot(target.getX(), target.getY());
        Log.v("IVK", "Target Dist: " + targetDist);
        double slideExtend = Math.sqrt(targetDist*targetDist - IVKConstants.clawOffsetUp*IVKConstants.clawOffsetUp) - IVKConstants.clawOffsetForward;
        Log.v("IVK", "Slide Extend: " + slideExtend);
        double slideTheta = Math.asin(-IVKConstants.clawOffsetUp/targetDist);
        Log.v("IVK", "Slide Theta: " + slideTheta);
        double theta = targetTheta + slideTheta;
        Log.v("IVK", "Theta: " + theta);
        return new Pose2d(slideExtend, 0, new Rotation2d(theta));
    }
    public static Command getIVKCommand(ExtensionSubsystem extension, PivotSubsystem pivot, Translation2d target) {
        Pose2d targets = getIVKClawPos(target);
        return new ParallelCommandGroup(
                new ExtendCommand(extension, targets.getX()),
                new PivotCommand(pivot, targets.getRotation().getDegrees())
        );
    }
    public static Command getIVKCommand(ExtensionSubsystem extension, PivotSubsystem pivot, Translation2d target, double speed) {
        Pose2d targets = getIVKClawPos(target);
        return new ParallelCommandGroup(
                new ClampExtendCommand(extension, targets.getX(), speed),
                new PivotCommand(pivot, targets.getRotation().getDegrees())
        );
    }

    public static Command getIVKCommand(ExtensionSubsystem extension, PivotSubsystem pivot, DoubleSupplier x, DoubleSupplier y) {
        Pose2d targets = getIVKClawPos(new Translation2d(x.getAsDouble(), y.getAsDouble()));
        return new ParallelCommandGroup(
                new ExtendCommand(extension, targets.getX()),
                new PivotCommand(pivot, targets.getRotation().getDegrees())
        );
    }


}
