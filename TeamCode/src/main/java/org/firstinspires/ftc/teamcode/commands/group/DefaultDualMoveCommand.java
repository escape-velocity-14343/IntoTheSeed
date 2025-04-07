package org.firstinspires.ftc.teamcode.commands.group;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinpointSubsystem;

public class DefaultDualMoveCommand extends CommandBase {

    private DefaultGoToPointCommand gtpc;
    private DefaultGVFCommand gvfc;
    public enum MoveState {
        P2P,
        GVF,
        BRAKE
    }

    private MoveState state = MoveState.P2P;
    private MecanumDriveSubsystem drive;

    public DefaultDualMoveCommand(MecanumDriveSubsystem driveSubsystem,
                                  PinpointSubsystem otosSubsystem,
                                  DefaultGoToPointCommand gtpc,
                                  DefaultGVFCommand gvfc) {
        addRequirements(driveSubsystem, otosSubsystem);
        this.gtpc = gtpc;
        this.gvfc = gvfc;
        this.drive = driveSubsystem;
    }

    @Override
    public void initialize() {
        gtpc.initialize();
        gvfc.initialize();
    }

    @Override
    public void execute() {
        switch (state) {
            case P2P:
                gtpc.execute();
                break;
            case GVF:
                gvfc.execute();
                break;
            case BRAKE:
                drive.driveFieldCentric(0, 0, 0);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        switch (state) {
            case P2P:
                gtpc.end(interrupted);
                break;
            case GVF:
                gvfc.end(interrupted);
                break;
        }

        Log.i("DualMove", "Default DualMove command finished.");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    public void setState(MoveState state) {
        this.state = state;
        if (state == MoveState.BRAKE) {
            drive.setBrake();
        } else {
            drive.clearBrake();
        }
    }

    public Command setP2P() {
        return new InstantCommand(() -> setState(MoveState.P2P));
    }

    public Command setGVF() {
        return new InstantCommand(() -> setState(MoveState.GVF));
    }

    public DefaultGoToPointCommand getGtpc() {
        return gtpc;
    }

    public DefaultGVFCommand getGvfc() {
        return gvfc;
    }

}
