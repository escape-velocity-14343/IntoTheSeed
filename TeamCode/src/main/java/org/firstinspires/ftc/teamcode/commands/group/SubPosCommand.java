package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Flips the wrist down after a SubPosReadyCommand.
 */
public class SubPosCommand extends SequentialCommandGroup {
    ExtensionSubsystem extension;

    public SubPosCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, PivotSubsystem pivot) {
        addCommands(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new PivotCommand(pivot, PivotConstants.intakePos)
        );
        // must require extension because manual control must use it, so this ensures any other commands using extension get interrupted
        addRequirements(extension, wrist);
        this.extension = extension;
    }
    public SubPosCommand(ExtensionSubsystem extension, WristSubsystem wrist, IntakeSubsystem intake, PivotSubsystem pivot, DoubleSupplier power) {
        addCommands(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new PivotCommand(pivot, PivotConstants.intakeReadyPos-power.getAsDouble()*(PivotConstants.intakeReadyPos-PivotConstants.intakePos)/0.55)
        );
        // must require extension because manual control must use it, so this ensures any other commands using extension get interrupted
        addRequirements(extension, wrist);
        this.extension = extension;
    }


    @Override
    public void end(boolean interrupted) {
        extension.setManualControl(true);
        Log.i("%6", "Sub Pos Command, Interrupted: " + interrupted);
    }
}
