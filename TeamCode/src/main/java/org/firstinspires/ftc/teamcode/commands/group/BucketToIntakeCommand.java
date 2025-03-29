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
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
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

public class BucketToIntakeCommand extends ParallelCommandGroup {

    public BucketToIntakeCommand(PivotSubsystem pivot, ExtensionSubsystem extension, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret, double forwardInches, double turretAngle, double offset) {
        super(
                new WristCommand(wrist, IntakeConstants.groundPos),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new TurretCommand(turret, turretAngle),
                new ExtendCommand(extension, SlideConstants.minExtension),
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> extension.getCurrentInches() < 5),
                        new PivotCommand(pivot, 30))
//                new InterruptCommand(
//                        new ParallelCommandGroup(
//                                new WristCommand(wrist, IntakeConstants.toptakePos),
//                                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 0),
//                                new TurretCommand(turret, turretAngle),
//                                new ExtendCommand(extension, Math.max(Math.min(Math.hypot(forwardInches, IVKCommand.intakeReadyY - IVKConstants.pivotPointHeight), SlideConstants.extendedThreshold - 2), SlideConstants.minExtension))
//                        ),
//                        () -> extension.getCurrentInches() < SlideConstants.minExtension
//                ),
        );
    }

}
