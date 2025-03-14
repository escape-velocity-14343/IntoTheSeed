package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.commands.custom.ExtendCommand;
import org.firstinspires.ftc.teamcode.commands.custom.InterruptCommand;
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

public class IntakeRetractCommand extends SequentialCommandGroup {
    private IntakeRetractCommand(Command... commands) {
        addCommands(commands);
    }

    public IntakeRetractCommand(
            WristSubsystem wrist,
            PivotSubsystem pivot,
            ExtensionSubsystem extend,
            TurretSubsystem turret) {
        ElapsedTime pivotTimer = new ElapsedTime();
        addCommands(
                new InstantCommand(pivotTimer::reset),
                new InterruptCommand(
                        new PivotCommand(pivot, PivotConstants.intakeReadyPos),
                        () ->
                                pivot.getPivotVelocity() < AutoConstants.autoscoreMaxPivotVel
                                        && pivotTimer.seconds() > 0.2),
                new WristCommand(wrist, IntakeConstants.foldedPos),
                new TurretCommand(turret, 0),
                new ParallelCommandGroup(new ExtendCommand(extend, SlideConstants.minExtension)),
                new WristCommand(wrist, IntakeConstants.foldedPos)
                        .whenFinished(() -> Log.i("5", "Intake Retract command")));
    }

    public static IntakeRetractCommand newWithWristPos(
            WristSubsystem wrist,
            PivotSubsystem pivot,
            ExtensionSubsystem extend,
            double wristPos) {
        return new IntakeRetractCommand(
                new ParallelCommandGroup(
                        new PivotCommand(pivot, PivotConstants.retractDegrees),
                        new ExtendCommand(extend, SlideConstants.minExtension)
                                .withTimeout(extend.getReasonableExtensionMillis(0))),
                new WristCommand(wrist, wristPos)
                        .whenFinished(() -> Log.i("%5", "Retract command")));
    }
}
