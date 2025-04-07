package org.firstinspires.ftc.teamcode.lib;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.security.InvalidParameterException;
import java.util.Collection;
import java.util.List;
import java.util.stream.Collectors;

public class Util {
    public static boolean inRange(double a, double b, double thres) {
        return Math.abs(a - b) < thres;
    }

    /**
     * @param target The target angle, in degrees
     * @param current The current angle, in degrees
     * @return Returns the <i>shortest</i> angular difference between target and current
     */
    public static double getAngularDifference(double target, double current) {
        return posmod(target - current + 180, 360) - 180;
    }

    /**
     * @param x X
     * @param y Y
     * @return Returns the floating-point modulus of x divided by y, wrapping equally in positive
     *     and negative
     */
    public static double posmod(double x, double y) {
        return x - Math.floor(x / y) * y;
    }

    public static double clamp(double max, double min, double v) {
        if (v > max) {
            return max;
        } else if (v < min) {
            return min;
        }
        return v;
    }

    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        } else {
            return value;
        }
    }

    public static double signedPower(double value, double power) {
        return Math.pow(Math.abs(value), power) * Math.signum(value);
    }

    public static double signedSqrt(double value) {
        return signedPower(value, 0.5);
    }

    public static Rotation2d angleToPoint(Pose2d current, Pose2d target) {
        Translation2d translation = target.getTranslation().minus(current.getTranslation());
        return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
    }

    public static Rotation2d angleToPoint(Translation2d current, Translation2d target) {
        Translation2d translation = target.minus(current);
        return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
    }

    public static double halfLinearHalfCubic(double input) {
        return (Math.pow(input, 3) + input) / 2;
    }

    public static double pose2dToDistance(Pose2d p1, Pose2d p2) {
        return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }

    public static double max(double... vals) {
        if (vals.length == 0) {
            throw new InvalidParameterException("Length cannot be zero");
        }
        double maxVal = vals[0];
        for (double val : vals) {
            maxVal = Math.max(maxVal, val);
        }
        return maxVal;
    }

    public static boolean isGamepadAlive(Gamepad gp, double threshold) {
        return Math.abs(gp.left_stick_x) > threshold
                || Math.abs(gp.left_stick_y) > threshold
                || Math.abs(gp.right_stick_x) > threshold
                || Math.abs(gp.right_stick_y) > threshold;
    }

    public static double mean(Collection<Double> values) {
        double total = 0.0;
        for (double value : values) {
            total += value;
        }
        return total / values.size();
    }

    public static double median(Collection<Double> values) {
        if (values.isEmpty()) {
            return 0.0;
        }
        if (values.size() == 1) {
            return values.toArray(new Double[0])[0];
        }
        List<Double> sortedValues = values.stream().sorted().collect(Collectors.toList());
        double middleValue = sortedValues.size() / 2.0 - 0.5;
        double floorMiddleValue = Math.floor(middleValue);
        if (floorMiddleValue != middleValue) {
            int intMiddleValue = (int)floorMiddleValue;
            double a = sortedValues.get(intMiddleValue);
            double b = sortedValues.get(intMiddleValue + 1);
            return (a + b) * 0.5;
        }

        return sortedValues.get((int) middleValue);
    }
}
