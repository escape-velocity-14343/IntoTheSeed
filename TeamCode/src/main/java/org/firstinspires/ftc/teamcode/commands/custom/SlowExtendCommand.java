package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

import java.util.function.DoubleSupplier;

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
    public SlowExtendCommand(ExtensionSubsystem extension, DoubleSupplier inches, double speed) {
        super(
                new InstantCommand(() -> extension.setManualControl(true)),
                extension.openloopC(speed),
                new WaitUntilCommand(() -> extension.getCurrentInches() > inches.getAsDouble()),
                extension.openloopC(0.0),
                new InstantCommand(() -> extension.setTargetInches(inches.getAsDouble())),
                new InstantCommand(() -> extension.setManualControl(false))
        );
        this.extension = extension;
        this.inches = inches.getAsDouble();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        extension.setManualControl(false);
        extension.setTargetInches(inches);
    }


}
