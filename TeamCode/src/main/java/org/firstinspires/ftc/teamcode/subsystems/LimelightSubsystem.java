package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util.cameraInfo;

import java.util.List;
import java.util.Optional;

public class LimelightSubsystem extends SubsystemBase {
    Limelight3A limelight;
    cameraInfo info;
    Optional<LLResult> lastResult = Optional.empty();

    /**
    * Initializes a new LimelightSubsystem
     * <br>Params:
     * @HardwareMap hMap
     */
    public LimelightSubsystem(HardwareMap hardwareMap, cameraInfo info){
        limelight = hardwareMap.get(Limelight3A.class, info.name());
        limelight.setPollRateHz(100); //times per second, polled
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void periodic(){
        //@MemoryLeak
        LLResult memoryLeak = limelight.getLatestResult();

        if (memoryLeak != null){
            if (memoryLeak.isValid()){
                double latency = memoryLeak.getCaptureLatency() + memoryLeak.getTargetingLatency(); //cl + tl = total latency
                lastResult = Optional.of(memoryLeak);
            }
        }
    }

    public Optional<List<LLResultTypes.DetectorResult>> getDetectorResults(){
        return lastResult.map(LLResult::getDetectorResults);
    }

    private long getStaleness(LLResult llResult){
        return llResult.getStaleness();
    }

    public cameraInfo getCameraInfo(){
        return info;
    }

}
