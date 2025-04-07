package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;

public class PolarIVKCommand extends ParallelCommandGroup {

    public PolarIVKCommand(PivotSubsystem pivot, ExtensionSubsystem extension, double angle, double magnitude) {
        super(
                new PivotCommand(pivot, angle),
                new ExtendCommand(extension, magnitude)
        );
    }

}
