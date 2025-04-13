package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IVKCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.RunIfCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
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
            DoubleSupplier forwardInches,
            BooleanSupplier notAlreadyInPosition) {

        addCommands(
                new RunIfCommand(
                        // first half of retract command
                        new ParallelCommandGroup(
                                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                                new ConditionalCommand(
                                        new WristCommand(wrist, (IntakeConstants.foldedPos + IntakeConstants.toptakePos) / 2),
                                        new WristCommand(wrist, IntakeConstants.foldedPos),
                                        () -> pivot.getCurrentPosition() > 70
                                ),
                                new TurretCommand(turret, 0),
                                new InterruptCommand(
                                        new ExtendCommand(extension, SlideConstants.minExtension),
                                        () -> extension.getCurrentInches() < SlideConstants.pivotDownExtension
                                )
                        ), notAlreadyInPosition
                ),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
                new SequentialCommandGroup(
                        new RunIfCommand(
                                new InterruptCommand(
                                        new PivotCommand(pivot, PivotConstants.bottomLimit),
                                        () -> pivot.getCurrentPosition() < 45.0
                                ), notAlreadyInPosition
                        ).andThen(
                                new IVKCommand(forwardInches, () -> IVKCommand.intakeReadyY, extension, pivot) {
                                    @Override
                                    public boolean isFinished() {
                                        return true;
                                    }
                                }
                        )
                ).alongWith(
                        new TurretCommand(turret, angle),
                        // flip down wrist to a ready position
                        new WaitUntilCommand(() -> Math.sin(pivot.getCurrentPosition()) * extension.getCurrentInches() > IVKCommand.intakeY - IVKConstants.pivotPointHeightOffset).andThen(new WristCommand(wrist, IntakeConstants.toptakePos))
                )

        );
    }

    public SubPosReadyCommand(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            IntakeSubsystem intake,
            TurretSubsystem turret,
            DoubleSupplier angle,
            double forwardInches,
            BooleanSupplier notAlreadyInPosition) {
        this(extension, pivot, wrist, intake, turret, angle, () -> forwardInches, notAlreadyInPosition);
    }
}
