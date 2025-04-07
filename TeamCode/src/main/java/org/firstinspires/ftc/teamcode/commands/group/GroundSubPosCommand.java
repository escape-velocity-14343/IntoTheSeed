package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class GroundSubPosCommand extends ParallelCommandGroup {
    public GroundSubPosCommand(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            double extensionTarget
    ) {
        addCommands(
                new TimeoutCommand(pivot.getPivotCommand(0.0), 0),
                new TimeoutCommand(extension.getExtendCommand(extensionTarget), 0),
                new TurretCommand(turret, 0.0),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, IntakeConstants.autoIntakeSpeed)
        );
    }
}
