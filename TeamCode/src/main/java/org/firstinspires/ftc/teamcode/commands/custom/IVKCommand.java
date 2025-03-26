package org.firstinspires.ftc.teamcode.commands.custom;

import androidx.core.math.MathUtils;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.robocol.Command;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.DoubleSupplier;

public class IVKCommand extends ParallelCommandGroup {
    //Control Equations
    private BiFunction<Double, Double, Double> getTargetExtension = (x, y) -> Math.max(Math.sqrt(Math.pow(x, 2) + Math.pow(y - IVKConstants.pivotPointHeight, 2)), 0);
    private BiFunction<Double, Double, Double> getTargetAngleDegrees = (x, y) -> Math.toDegrees(Math.atan2(y - IVKConstants.pivotPointHeight, x));

    // useful constants:
    public static double intakeReadyY = 9;
    public static double intakeY = 7;

    /**
     * Height is from the tile to the claw
     * Distance is from the front of the robot, to a point forwards from the bot
     *
     * Units are in inches
     *
     * Schedule intake position command before this, to prevent samples from getting hit
     */
    public IVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem){
        addCommands(
                extensionSubsystem.getExtendCommand(getTargetExtension.apply(x, y)),
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees.apply(x, y))
        );
    }

    public IVKCommand(DoubleSupplier x, DoubleSupplier y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem){
        addCommands(
                extensionSubsystem.getExtendCommand(getTargetExtension.apply(x.getAsDouble(), y.getAsDouble())),
                pivotSubsystem.getPivotCommand(getTargetAngleDegrees.apply(x.getAsDouble(), y.getAsDouble())),
                new InstantCommand(() -> System.out.println(getTargetExtension.apply(x.getAsDouble(), y.getAsDouble()))),
                new InstantCommand(() -> System.out.println(getTargetAngleDegrees.apply(x.getAsDouble(), y.getAsDouble())))
        );
    }
}
