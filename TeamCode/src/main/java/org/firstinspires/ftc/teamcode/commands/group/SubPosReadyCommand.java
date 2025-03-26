package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeSpinCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.RunIfCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command for extending slides out and half flipping claw down, but not yet fully bringing the claw
 * down.
 */
public class SubPosReadyCommand extends SequentialCommandGroup {

    public SubPosReadyCommand(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            DoubleSupplier angle,
            double forwardInches,
            BooleanSupplier notAlreadyInPosition) {

        addCommands(
                new RunIfCommand(new RetractCommand(wrist, pivot, extension, turret, intake), notAlreadyInPosition),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                new IVKCommand(forwardInches, IVKCommand.intakeReadyY, extension, pivot).alongWith(
                        new TurretCommand(turret, angle)
                ),
                // flip down wrist to a ready position
                new WristCommand(wrist, IntakeConstants.toptakePos)

        );
    }
}
