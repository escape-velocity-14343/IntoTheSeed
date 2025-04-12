package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

public class ClampExtendCommand extends SequentialCommandGroup {

    private ExtensionSubsystem extension;
    public ClampExtendCommand(ExtensionSubsystem extension, double inches, double maxPower) {
        super(
                new InstantCommand(() -> extension.setMaxPower(maxPower)),
                new ExtendCommand(extension, inches),
                new InstantCommand(() -> extension.setMaxPower(1.0))
        );
        this.extension = extension;
    }

    @Override
    public void end(boolean interrupted) {
        extension.setMaxPower(1.0);
    }

}
