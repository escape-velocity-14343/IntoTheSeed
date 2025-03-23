package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Objects;
import java.util.function.Supplier;

import javax.annotation.Nullable;

public class TimeoutCommand extends CommandBase {

    private Command command;
    private ElapsedTime timer;
    private int timeoutMs;
    @Nullable
    private Supplier<Integer> timeoutMsSupplier = null;

    public TimeoutCommand(Command command, int timeoutMs) {
        this.command = command;
        this.timer = new ElapsedTime();
        this.timeoutMs = timeoutMs;
    }

    public TimeoutCommand(Command command, Supplier<Integer> timeoutMsSupplier) {
        this.command = command;
        this.timer = new ElapsedTime();
        this.timeoutMsSupplier = timeoutMsSupplier;
    }

    @Override
    public void initialize() {
        if (Objects.nonNull(timeoutMsSupplier)) {
            timeoutMs = timeoutMsSupplier.get();
        }
        command.initialize();
        timer.reset();
    }

    @Override
    public void execute() {
        command.execute();
    }

    @Override
    public void end(boolean interrupted) {
        // if we hit timeout, it also interrupts
        command.end(interrupted || timer.milliseconds() > timeoutMs);
    }

    @Override
    public boolean isFinished() {
        return command.isFinished() || timer.milliseconds() > timeoutMs;
    }
}
