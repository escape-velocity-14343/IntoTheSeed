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
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.constants.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class BucketPos2Command extends SequentialCommandGroup {
    private BucketPos2Command(Command... commands) {
        super(commands);
    }

    public BucketPos2Command(
            ExtensionSubsystem extension,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            TurretSubsystem turret, boolean auto) {
        addCommands(
                // new ExtendCommand(extension, 1),
                new ParallelCommandGroup(

                        // minus two to prevent it from overshooting
                        new PivotCommand(pivot, PivotConstants.stallTopLimit)
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
                                                        > (auto ? PivotConstants.autoOuttakeExtendDegrees : PivotConstants.outtakeExtendDegrees)),
                                new ExtendCommand(
                                        extension,
                                        SlideConstants.bucketPos
                                                + (SlideConstants.highExtend
                                                ? SlideConstants.highExtendInches
                                                : 0)
                                                + (SlideConstants.lowExtend
                                                ? SlideConstants.lowExtendInches
                                                : 0))
                                        .withTimeout(1000))
                                .alongWith(
                                        new WristCommand(wrist, IntakeConstants.scoringPos),
                                        new TurretCommand(turret, 90)
                                ),
                new InstantCommand(() -> Log.i("%2", "BucketPos End"))));
    }
}