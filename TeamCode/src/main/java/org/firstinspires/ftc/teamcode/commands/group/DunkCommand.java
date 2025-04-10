package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TimeoutCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class DunkCommand extends SequentialCommandGroup {
    private DunkCommand(Command... commands) {
        super(commands);
    }

    public DunkCommand(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            TurretSubsystem turret,
            IntakeSubsystem intake) {
        addCommands(
                new WristCommand(wrist, IntakeConstants.groundPos),
                new TurretCommand(turret, 0),
                // minus two to prevent it from overshooting
                new ParallelCommandGroup(
                        new PivotCommand(pivot, PivotConstants.stallTopLimit)
                                .interruptOn(
                                        () ->
                                                pivot.getPivotVelocity()
                                                        < AutoConstants.autoscoreMaxPivotVel
                                                        && pivot.getCurrentPosition()
                                                        > PivotConstants.topLimit - 4),
                        new WaitUntilCommand(
                                () -> pivot.getCurrentPosition() > PivotConstants.autoOuttakeExtendDegrees
                        ).andThen(
                                new ParallelCommandGroup(
                                        new InterruptCommand(
                                        new ExtendCommand(
                                                extension,
                                                SlideConstants.bucketPos
                                                        + (SlideConstants.highExtend
                                                        ? SlideConstants.highExtendInches
                                                        : 0)
                                                        + (SlideConstants.lowExtend
                                                        ? SlideConstants.lowExtendInches
                                                        : 0)),
                                                () -> extension.getCurrentInches() > SlideConstants.safeForDunk),
                                        new WaitUntilCommand(
                                                () -> extension.getCurrentInches() > SlideConstants.safeForDunk
                                        ).andThen(new WristCommand(wrist, IntakeConstants.dunkScoringPos).interruptOn(intake::stable))
                                )
                        )
                ),
                new ParallelRaceGroup(new TimeoutCommand(new RunCommand(()->Log.i("imu poo", "imu accel: " + intake.getAccel())), 50), new WaitUntilCommand(intake::stable)),
                new IntakeClawCommand(intake, IntakeConstants.openPos),
                new InstantCommand(() -> Log.i("%2", "Dunk End")));
    }
}