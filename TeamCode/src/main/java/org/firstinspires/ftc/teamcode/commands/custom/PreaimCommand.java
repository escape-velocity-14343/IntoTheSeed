package org.firstinspires.ftc.teamcode.commands.custom;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.group.DefaultGoToPointCommand;
import org.firstinspires.ftc.teamcode.commands.group.GoToPointWithDefaultCommand;
import org.firstinspires.ftc.teamcode.constants.IVKConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PivotSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TargetingSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

import java.util.function.BiFunction;

public class PreaimCommand extends ParallelCommandGroup {

    public PreaimCommand(DefaultGoToPointCommand gtpc, PivotSubsystem pivot, ExtensionSubsystem extension, IntakeSubsystem intake, WristSubsystem wrist, TurretSubsystem turret, TargetingSubsystem target, PinpointSubsystem pinpoint) {
        addCommands(
                new IVKSample(target, pivot, extension),
                new IntakeControlCommand(intake, IntakeConstants.singleIntakePos, 1),
                new WristCommand(wrist, IntakeConstants.toptakePos),
                new TurretCommand(turret, target::getAngle),
                new InterruptCommand(
                        new GoToPointWithDefaultCommand(target::getDBTarget, gtpc, 1.0, 2.0),
                        () -> Math.abs(target.getDBTarget().getX() - pinpoint.getPose().getX()) < 0.5
                )
        );
    }

    public class IVKSample extends CommandBase {
        TargetingSubsystem target;
        PivotSubsystem pivot;
        ExtensionSubsystem extension;

        public IVKSample(TargetingSubsystem target, PivotSubsystem pivot, ExtensionSubsystem extension) {
            this.target = target;
            this.pivot = pivot;
            this.extension = extension;
        }

        @Override
        public void initialize() {
            double x = Math.abs(target.getX()) + 17;
            double y = target.getY();
            CommandScheduler.getInstance().schedule(new SequentialIVKCommand(x, IVKConstants.neutralY, extension, pivot));
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class IVKGround extends CommandBase {
        TargetingSubsystem target;
        PivotSubsystem pivot;
        ExtensionSubsystem extension;

        public IVKGround(TargetingSubsystem target, PivotSubsystem pivot, ExtensionSubsystem extension) {
            this.target = target;
            this.pivot = pivot;
            this.extension = extension;
        }

        @Override
        public void initialize() {
            double x = Math.abs(target.getX()) + 18;
            double y = target.getY();
            CommandScheduler.getInstance().schedule(new SequentialIVKCommand(x, 2, extension, pivot));
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /*public class IVKCommand extends SequentialCommandGroup {
        //Control Equations
        public BiFunction<Double, Double, Double> getTopTriangleAngle = (x, y) -> Math.toDegrees(
                Math.asin(
                        IVKConstants.intakeLength
                                / Math.sqrt(
                                Math.pow(y - IVKConstants.pivotPointHeight, 2)
                                        + Math.pow(x, 2))));
        public BiFunction<Double, Double, Double> getTargetExtension = (x, y) -> 1.0 / Math.tan(Math.toRadians(getTopTriangleAngle.apply(x, y))) * IVKConstants.intakeLength - 12;
        public BiFunction<Double, Double, Double> getTargetAngleDegrees = (x, y) ->
                Math.toDegrees(Math.atan2(y - IVKConstants.pivotPointHeight, x))
                        + getTopTriangleAngle.apply(x, y);

        /**
         * Height is from the tile to the claw
         * Distance is from the front of the robot, to a point forwards from the bot
         * <p>
         * Units are in inches
         * <p>
         * Schedule intake position command before this, to prevent samples from getting hit
         */
        /*public IVKCommand(double x, double y, ExtensionSubsystem extensionSubsystem, PivotSubsystem pivotSubsystem) {
            addCommands(
                    pivotSubsystem.getPivotCommand(getTargetAngleDegrees.apply(x, y)),
                    extensionSubsystem.getExtendCommand(getTargetExtension.apply(x, y))
            );
        }
    }*/
}
