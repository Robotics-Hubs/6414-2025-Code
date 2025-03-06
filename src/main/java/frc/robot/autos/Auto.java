package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.utils.FieldMirroringUtils;

public class Auto {
    private static boolean isRightSide;

    public void setIsRightSide(boolean rightSide) {
        isRightSide = rightSide;
    }

    public static boolean getRightSide() {
        return isRightSide;
    }

    public static Pose2d flipLeftRight(Pose2d pose) {
        return new Pose2d(
                pose.getX(),
                FieldMirroringUtils.FIELD_HEIGHT - pose.getY(),
                pose.getRotation().unaryMinus());
    }
}
