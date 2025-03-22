package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.RunIfCommand;
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
            double angle,
            double extendInches,
            BooleanSupplier notAlreadyInPosition) {

        addCommands(
                new RunIfCommand(new RetractCommand(wrist, pivot, extension, turret, intake), notAlreadyInPosition),
                new PivotCommand(pivot, PivotConstants.intakeReadyPos),
                new ExtendCommand(extension, extendInches),
//                        .withTimeout(extension.getReasonableExtensionMillis(extendInches)),
                // flip down wrist to a ready position
                new WristCommand(wrist, IntakeConstants.toptakePos)
                        .alongWith(new TurretCommand(turret, angle)));
    }
}
