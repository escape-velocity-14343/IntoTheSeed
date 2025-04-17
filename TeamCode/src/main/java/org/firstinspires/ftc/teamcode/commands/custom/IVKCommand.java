package org.firstinspires.ftc.teamcode.commands.custom;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.function.DoubleSupplier;

@Config
public class IVKCommand extends ParallelCommandGroup {
    // useful constants:
    // DO NOT MODIFY; MODIFY THE ONES IN IVKCONSTANTS
    // kept for backcompatibility
    public static double intakeReadyY = IVKConstants.intakeReadyY;
    public static double intakeY = IVKConstants.intakeY;

    /**
     * Height is from the tile to the claw
     * Distance is from the front of the robot, to a point forwards from the bot
     * <p>
     * Units are in inches
     * <p>
     * Schedule intake position command before this, to prevent samples from getting hit
     */
    public IVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                extensionSubsystem.getExtendCommand(getTargetExtension(x, y)),
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x, y))
        );
    }
    public IVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, double power) {
        addCommands(
                new ClampExtendCommand(extensionSubsystem, getTargetExtension(x, y),power),
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x, y))
        );
    }


    public IVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem) {
        addCommands(
                extensionSubsystem.getExtendCommand(() -> getTargetExtension(x.getAsDouble(), y.getAsDouble())),
                pivotSubsystem.getPivotCommand(() -> getTargetAngleDegrees(x.getAsDouble(), y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension(x.getAsDouble(), y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees(x.getAsDouble(), y.getAsDouble())))
        );
    }
    public IVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, double speed) {
        addCommands(
                new SlowExtendCommand(extensionSubsystem, () -> getTargetExtension(x.getAsDouble(), y.getAsDouble()), speed),
                pivotSubsystem.getPivotCommand(() -> getTargetAngleDegrees(x.getAsDouble(), y.getAsDouble())),
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
    public IVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, int a) {
        addCommands(
                extensionSubsystem.getExtendCommand(getTargetExtension(x + IVKConstants.ivkCenterOffset, y)),
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x + IVKConstants.ivkCenterOffset, y))
        );
    }

    public IVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, int a) {
        addCommands(
                extensionSubsystem.getExtendCommand(() -> getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                pivotSubsystem.getPivotCommand(() -> getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())))
        );
    }

    public IVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem, String a) {
        addCommands(
                extensionSubsystem.getExtendCommand(getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees(x.getAsDouble() + IVKConstants.ivkCenterOffset, y.getAsDouble())))
        );
    }

    private double getTargetExtension(double x, double y) {
        return Math.max(Math.sqrt(Math.pow(x, 2) + Math.pow(y - IVKConstants.pivotPointHeightOffset, 2)), 0);
    }

    private double getTargetAngleDegrees(double x, double y) {
        return Math.toDegrees(Math.atan2(y - IVKConstants.pivotPointHeightOffset, x));
    }
}
