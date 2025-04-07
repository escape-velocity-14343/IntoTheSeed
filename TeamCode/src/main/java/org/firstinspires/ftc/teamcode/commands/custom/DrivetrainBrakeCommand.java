package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.group.DefaultDualMoveCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class DrivetrainBrakeCommand extends InstantCommand {

    public DrivetrainBrakeCommand(DefaultDualMoveCommand dmc) {
        super(() -> dmc.setState(DefaultDualMoveCommand.MoveState.BRAKE));
    }

}
