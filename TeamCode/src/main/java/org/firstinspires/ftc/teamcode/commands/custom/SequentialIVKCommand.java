package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.function.DoubleSupplier;

public class SequentialIVKCommand extends SequentialCommandGroup {
    // useful constants:
    public static double intakeReadyY = 10;
    public static double intakeY = 7;

    /**
     * Height is from the tile to the claw
     * Distance is from the front of the robot, to a point forwards from the bot
     * <p>
     * Units are in inches
     * <p>
     * Schedule intake position command before this, to prevent samples from getting hit
     */
    public SequentialIVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x, y)),
                extensionSubsystem.getExtendCommand(getTargetExtension(x, y))
        );
    }

    public SequentialIVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                pivotSubsystem.getPivotCommand(() -> getTargetAngleDegrees(x.getAsDouble(), y.getAsDouble())),
                extensionSubsystem.getExtendCommand(() -> getTargetExtension(x.getAsDouble(), y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension(x.getAsDouble(), y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees(x.getAsDouble(), y.getAsDouble())))
        );
    }

    /**
     * The origin of x is the center of the bot
     *
     * @param x
     * @param y
     * @param extensionSubsystem
     * @param pivotSubsystem
     */
    public SequentialIVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, int a) {
        addCommands(
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x + IVKConstants.ivkCenterOffset, y)),
                extensionSubsystem.getExtendCommand(getTargetExtension(x + IVKConstants.ivkCenterOffset, y))
        );
    }

    public SequentialIVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, int a) {
        addCommands(
                pivotSubsystem.getPivotCommand(() -> getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                extensionSubsystem.getExtendCommand(() -> getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())))
        );
    }

    public SequentialIVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, String a) {
        addCommands(
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                extensionSubsystem.getExtendCommand(getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())))
        );
    }

    private double getTargetExtension(double x, double y) {
        return Math.max(Math.sqrt(Math.pow(x, 2) + Math.pow(y - IVKConstants.pivotPointHeightCam, 2)), 0);
    }

    private double getTargetAngleDegrees(double x, double y) {
        return Math.toDegrees(Math.atan2(y - IVKConstants.pivotPointHeightCam, x));
    }
}
