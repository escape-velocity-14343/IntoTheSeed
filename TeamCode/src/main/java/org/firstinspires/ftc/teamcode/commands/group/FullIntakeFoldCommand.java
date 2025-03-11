package org.firstinspires.ftc.teamcode.commands.group;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.custom.IntakeControlCommand;
import org.firstinspires.ftc.teamcode.commands.custom.TurretCommand;
import org.firstinspires.ftc.teamcode.commands.custom.WristCommand;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class FullIntakeFoldCommand extends ParallelCommandGroup {

    public FullIntakeFoldCommand(IntakeSubsystem intake, TurretSubsystem turret, WristSubsystem wrist) {
        super(
                new IntakeControlCommand(intake, IntakeConstants.closedPos, 0),
                new TurretCommand(turret, 0),
                new WristCommand(wrist, IntakeConstants.foldedPos)
        );
    }

}
