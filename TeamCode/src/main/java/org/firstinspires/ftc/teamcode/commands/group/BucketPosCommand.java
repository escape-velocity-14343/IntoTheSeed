package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BucketPosCommand extends SequentialCommandGroup {
    private BucketPosCommand(Command... commands) {
        super(commands);
    }

    public BucketPosCommand(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            TurretSubsystem turret) {
        addCommands(
                // new ExtendCommand(extension, 1),
                new ParallelCommandGroup(
                        // minus two to prevent it from overshooting
                        new PivotCommand(pivot, PivotConstants.topLimit)
                                .interruptOn(
                                        () ->
                                                pivot.getPivotVelocity()
                                                                < AutoConstants.autoscoreMaxPivotVel
                                                        && pivot.getCurrentPosition()
                                                                > PivotConstants.topLimit - 4),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(
                                        () ->
                                                pivot.getCurrentPosition()
                                                        > PivotConstants.outtakeExtendDegrees),
                                new ExtendCommand(
                                                extension,
                                                SlideConstants.bucketPos
                                                        + (DriveConstants.highExtend
                                                                ? SlideConstants.highExtendInches
                                                                : 0))
                                        .withTimeout(1000)
                                        .interruptOn(
                                                () ->
                                                        extension.getCurrentInches()
                                                                > SlideConstants.bucketPos - 2)),
                        new WaitUntilCommand(
                                        () ->
                                                extension.getCurrentInches()
                                                        > SlideConstants.bucketPos - 2)
                                .withTimeout(1000)
                                .andThen(new WristCommand(wrist, IntakeConstants.scoringPos)),
                        new TurretCommand(turret, 0)),
                new InstantCommand(() -> Log.i("%2", "BucketPos End")));
    }

    public static BucketPosCommand newWithWristPos(
            ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        return new BucketPosCommand(
                new ParallelCommandGroup(
                        // minus two to prevent it from overshooting
                        new WristCommand(wrist, IntakeConstants.scoringPos),
                        new PivotCommand(pivot, PivotConstants.topLimit - 3),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(
                                        () ->
                                                pivot.getCurrentPosition()
                                                        > PivotConstants.outtakeExtendDegrees),
                                new ExtendCommand(extension, SlideConstants.bucketPos)
                                        .withTimeout(1000))),
                new WristCommand(wrist, IntakeConstants.scoringPos),
                new InstantCommand(() -> Log.i("%2", "BucketPos End")));
    }
}
