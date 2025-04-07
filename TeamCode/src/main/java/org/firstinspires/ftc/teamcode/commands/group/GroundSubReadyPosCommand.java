package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class GroundSubReadyPosCommand extends ParallelCommandGroup {
    public GroundSubReadyPosCommand(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            double extensionTarget
    ) {
        addCommands(
                pivot.getPivotCommand(0.0),
                extension.getExtendCommand(extensionTarget),
                new TurretCommand(turret, 0.0),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, IntakeConstants.autoIntakeSpeed)
        );
    }
}
