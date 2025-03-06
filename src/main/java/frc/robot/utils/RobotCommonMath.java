package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Random;

public class RobotCommonMath {
    private static final Random random = new Random();

    public RobotCommonMath() {
    }

    public static double generateRandomNormal(double mean, double stdDev) {
        double u1 = random.nextDouble();
        double u2 = random.nextDouble();
        double z0 = Math.sqrt(-2.0 * Math.log(u1)) * Math.cos(6.283185307179586 * u2);
        return z0 * stdDev + mean;
    }

    public static double constrainMagnitude(double value, double maxMagnitude) {
        return Math.copySign(Math.min(Math.abs(value), Math.abs(maxMagnitude)), value);
    }

    public static double linearInterpretationWithBounding(double x1, double y1, double x2, double y2, double x) {
        double minX = Math.min(x1, x2);
        double maxX = Math.max(x1, x2);
        return linearInterpretation(x1, y1, x2, y2, Math.min(maxX, Math.max(minX, x)));
    }

    public static double linearInterpretation(double x1, double y1, double x2, double y2, double x) {
        return y1 + (x - x1) * (y2 - y1) / (x2 - x1);
    }

    public static Rotation2d getAngle(Translation2d translation2d) {
        double tooSmall = 1.0E-6;
        return translation2d.getNorm() < 1.0E-6 ? Rotation2d.fromDegrees(0.0) : translation2d.getAngle();
    }
}
