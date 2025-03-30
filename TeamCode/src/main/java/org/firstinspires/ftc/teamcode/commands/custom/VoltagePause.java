package org.firstinspires.ftc.teamcode.commands.custom;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.lib.CachingVoltageSensor;

public class VoltagePause extends WaitCommand {

    /**
     * overload for side to side stability
     * @param voltageSensor
     */
    public VoltagePause(CachingVoltageSensor voltageSensor){
//        super((long) ((voltageSensor.getVoltage()-AutoConstants.baseVoltage)*0 + 0));
        super(0);
    }

    /**
     * overload for front to back stability
     */
    public VoltagePause(CachingVoltageSensor voltageSensor, int meaninglessValue){
//        super((long) ((voltageSensor.getVoltage()-AutoConstants.baseVoltage)*0 + 0));
        super(0);
    }
}
