package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.IntakeClawCommand;
import org.firstinspires.ftc.teamcode.commands.custom.PivotCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.PivotConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

// AUTO ONLY
public class IntakePosCommand extends SequentialCommandGroup {
    public IntakePosCommand(
            ExtensionSubsystem extend,
            PivotSubsystem pivot,
            WristSubsystem wrist,
            IntakeSubsystem intake) {
        addCommands(
                // new ParallelCommandGroup(
                new ExtendCommand(extend, 1)
                        .interruptOn(() -> extend.getCurrentInches() < 3)
                        .alongWith(
                                new WaitUntilCommand(() -> extend.getCurrentInches() < 25)
                                        .andThen(
                                                new IntakeClawCommand(
                                                        intake, IntakeConstants.singleIntakePos),
                                                new WristCommand(wrist, IntakeConstants.groundPos),
                                                new PivotCommand(
                                                        pivot, PivotConstants.retractDegrees))),
                new WristCommand(wrist, IntakeConstants.groundPos)
                        .whenFinished(
                                () ->
                                        Log.i(
                                                "intake pos command",
                                                "pivot pos: " + pivot.getCurrentPosition())));
    }

    @Deprecated
    // FOR BACKCOMPAT ONLY DO NOT MODIFY
    public IntakePosCommand(ExtensionSubsystem extend, PivotSubsystem pivot, WristSubsystem wrist) {
        addCommands(
                // new ParallelCommandGroup(
                new ExtendCommand(extend, 1)
                        .interruptOn(() -> extend.getCurrentInches() < 5)
                        .alongWith(
                                new WaitUntilCommand(() -> extend.getCurrentInches() < 25)
                                        .andThen(
                                                new WristCommand(wrist, IntakeConstants.groundPos),
                                                new PivotCommand(
                                                                pivot,
                                                                PivotConstants.retractDegrees)
                                                        .interruptOn(
                                                                () ->
                                                                        pivot.getCurrentPosition()
                                                                                < 4))),
                new WristCommand(wrist, IntakeConstants.groundPos)
                        .whenFinished(
                                () ->
                                        Log.i(
                                                "intake pos command",
                                                "pivot pos: " + pivot.getCurrentPosition())));
    }
}
