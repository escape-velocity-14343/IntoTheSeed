package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class SpecimenRaiseCommand extends SequentialCommandGroup {
    private ExtensionSubsystem extend;

    public SpecimenRaiseCommand(PivotSubsystem pivot, ExtensionSubsystem extension, WristSubsystem wrist) {
        this.extend = extension;

        addCommands(
                new ParallelCommandGroup(
                    new WristCommand(wrist, IntakeConstants.specimenReadyPos),
                    new PivotCommand(pivot, PivotConstants.specimenIntakeAngle),
                    new ExtendCommand(extension, SlideConstants.specimenRaisePosition).withTimeout(extension.getReasonableExtensionMillis(SlideConstants.specimenRaisePosition))
                ),
                new InstantCommand(() -> extension.setManualControl(true)),
                new InstantCommand(() -> extension.openloop(-1)),
                new WaitCommand(300),
                new InstantCommand(() -> extension.openloop(0)),
                new InstantCommand(() -> extension.setManualControl(false))

        );
    }

    @Override
    public void end(boolean interrupted) {
        extend.setManualControl(false);
    }
}
