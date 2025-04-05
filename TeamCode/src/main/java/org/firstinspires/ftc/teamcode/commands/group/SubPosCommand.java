package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import java.util.function.DoubleSupplier;

import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

/** Flips the wrist down after a SubPosReadyCommand. */
public class SubPosCommand extends SequentialCommandGroup {
    ExtensionSubsystem extension;

    public SubPosCommand(
            ExtensionSubsystem extension,
            WristSubsystem wrist,
            IntakeSubsystem intake,
            PivotSubsystem pivot,
            double forwardExtension) {
        addCommands(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new IVKCommand(forwardExtension, IVKCommand.intakeY, extension, pivot) {
                    @Override
                    public boolean isFinished() {
                        return true;
                    }
                }
        );
        addRequirements(wrist);
        this.extension = extension;
    }

    public SubPosCommand(
            ExtensionSubsystem extension,
            WristSubsystem wrist,
            IntakeSubsystem intake,
            PivotSubsystem pivot,
            DoubleSupplier forwardExtension) {
        addCommands(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new IVKCommand(forwardExtension, () -> IVKCommand.intakeY, extension, pivot)
        );
        addRequirements(wrist);
        this.extension = extension;
    }

    public SubPosCommand(
            ExtensionSubsystem extension,
            WristSubsystem wrist,
            IntakeSubsystem intake,
            PivotSubsystem pivot,
            Double forwardExtension,
            Double intakeY) {
        addCommands(
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new IVKCommand(forwardExtension, intakeY, extension, pivot)
        );
        addRequirements(wrist);
        this.extension = extension;
    }

    @Override
    public void end(boolean interrupted) {
//        extension.setManualControl(true);
        Log.i("%6", "Sub Pos Command, Interrupted: " + interrupted);
    }
}
