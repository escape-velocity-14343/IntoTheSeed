package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.DoubleSupplier;

public class TurretCommand extends InstantCommand {

    public TurretCommand(TurretSubsystem turret, double angle) {
        super(() -> turret.rotateTo(angle));
    }

    public TurretCommand(TurretSubsystem turret, DoubleSupplier angleSupplier){
        super(() -> turret.rotateTo(angleSupplier.getAsDouble()));
    }

}
