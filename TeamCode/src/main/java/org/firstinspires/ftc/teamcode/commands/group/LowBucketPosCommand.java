package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class LowBucketPosCommand extends SequentialCommandGroup {
    private LowBucketPosCommand(Command... commands) {
        super(commands);
    }

    public LowBucketPosCommand(
            ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                // new ExtendCommand(extension, 1),
                new WristCommand(wrist, IntakeConstants.groundPos),
                new ParallelCommandGroup(
                        // minus two to prevent it from overshooting
                        new PivotCommand(pivot, PivotConstants.topLimit - 3),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(
                                        () ->
                                                pivot.getCurrentPosition()
                                                        > PivotConstants.outtakeExtendDegrees),
                                new ExtendCommand(extension, SlideConstants.lowBucketPos)
                                        .withTimeout(1000))),
                new WristCommand(wrist, IntakeConstants.scoringPos),
                new InstantCommand(() -> Log.i("2", "BucketPos End")));
    }

    public static LowBucketPosCommand newWithWristPos(
            ExtensionSubsystem extension, PivotSubsystem pivot, WristSubsystem wrist) {
        return new LowBucketPosCommand(
                new ParallelCommandGroup(
                        // minus two to prevent it from overshooting
                        new WristCommand(wrist, IntakeConstants.scoringPos),
                        new PivotCommand(pivot, PivotConstants.topLimit - 3),
                        new SequentialCommandGroup(
                                new WaitUntilCommand(
                                        () ->
                                                pivot.getCurrentPosition()
                                                        > PivotConstants.outtakeExtendDegrees),
                                new ExtendCommand(extension, SlideConstants.lowBucketPos)
                                        .withTimeout(1000))),
                new WristCommand(wrist, IntakeConstants.scoringPos),
                new InstantCommand(() -> Log.i("2", "BucketPos End")));
    }
}
