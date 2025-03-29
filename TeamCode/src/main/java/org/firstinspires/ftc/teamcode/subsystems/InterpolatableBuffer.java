package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;
import java.util.function.Consumer;

public class InterpolatableBuffer extends SubsystemBase {
    private PinpointSubsystem pinpoint;
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //Internal Timestamp, Pose
    private TreeMap<Double, Pose2d> internalBuffer = new TreeMap<>();
    private Consumer<Pose2d> consumerReference = this::consumePose;
    private Double ageLimit = Double.POSITIVE_INFINITY;
    private double cleanCount = 0;

    public InterpolatableBuffer(PinpointSubsystem pinpoint, Double ageLimitMilliseconds){
        this.pinpoint = pinpoint;
        this.ageLimit = ageLimitMilliseconds;

        timer.reset();
    }

    @Override
    public void periodic() {
        cleanCount++;
        consumePose(pinpoint.getPose());

        if (cleanCount > 4){
            cleanCount = 0;
            cleanUp();
        }
    }

    public void consumePose(Pose2d pose){
        internalBuffer.put(timer.milliseconds(), pose);
    }

    public void cleanUp(){
        internalBuffer.headMap(timer.milliseconds() - ageLimit, true).clear();
    }

    public void clearAll(){
        internalBuffer.clear();
    }

    /**
     * For use with camera processing
     * @param latency
     * @return
     */
    public Optional<Pose2d> getPastSample(Double latency){
        Double time = timer.milliseconds();

        if (internalBuffer.isEmpty()){
            return Optional.empty();
        }

        Map.Entry<Double, Pose2d> topBound = internalBuffer.ceilingEntry(time-latency);
        Map.Entry<Double, Pose2d> bottomBound = internalBuffer.floorEntry(time-latency);

        if (topBound == null && bottomBound == null) {
            return Optional.empty();
        } else if (topBound == null) {
            return Optional.of(bottomBound.getValue());
        } else if (bottomBound == null) {
            return Optional.of(topBound.getValue());
        }
        else{
            return Optional.of(average(topBound.getValue(), bottomBound.getValue()));
        }
    }

    private Pose2d average(Pose2d first, Pose2d second){
        double delta = ((second.getHeading() - first.getHeading() + Math.PI) % (2 * Math.PI)) - Math.PI;

        return new Pose2d(
                (first.getX() + second.getX())/2,
                (first.getY() + second.getY())/2,
                new Rotation2d(first.getHeading() + 0.5 * delta));
    }
}
