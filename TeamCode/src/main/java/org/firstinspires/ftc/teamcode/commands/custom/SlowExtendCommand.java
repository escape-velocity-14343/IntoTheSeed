package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

public class SlowExtendCommand extends SequentialCommandGroup {

    ExtensionSubsystem extension;
    double inches;

    public SlowExtendCommand(ExtensionSubsystem extension, double inches, double speed) {
        super(
                new InstantCommand(() -> extension.setManualControl(true)),
                extension.openloopC(speed),
                new WaitUntilCommand(() -> extension.getCurrentInches() > inches),
                extension.openloopC(0.0),
                new InstantCommand(() -> extension.setTargetInches(inches)),
                new InstantCommand(() -> extension.setManualControl(false))
        );
        this.extension = extension;
        this.inches = inches;
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        extension.setManualControl(false);
        extension.setTargetInches(inches);
    }


}
