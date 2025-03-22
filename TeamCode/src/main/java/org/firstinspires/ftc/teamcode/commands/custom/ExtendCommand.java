package org.firstinspires.ftc.teamcode.commands.custom;

import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;

public class ExtendCommand extends TimeoutCommand {

    /**
     * @param subsystem
     * @param target    in inches
     */
    public ExtendCommand(ExtensionSubsystem subsystem, double target) {
        super(new ExtendCommandInternal(subsystem, target), (int) subsystem.getReasonableExtensionMillis(target));
    }

}
