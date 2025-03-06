package frc.robot.utils;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;

public class FieldMirroringUtils {
    public static final double FIELD_WIDTH = 17.548;
    public static final double FIELD_HEIGHT = 8.052;

    public FieldMirroringUtils() {
    }

    public static Rotation2d toCurrentAllianceRotation(Rotation2d rotationAtBlueSide) {
        return isSidePresentedAsRed() ? flip(rotationAtBlueSide) : rotationAtBlueSide;
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.plus(Rotation2d.k180deg);
    }

    public static Translation2d toCurrentAllianceTranslation(Translation2d translationAtBlueSide) {
        return isSidePresentedAsRed() ? flip(translationAtBlueSide) : translationAtBlueSide;
    }

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(17.548 - translation.getX(), 8.052 - translation.getY());
    }

    public static Translation3d toCurrentAllianceTranslation(Translation3d translation3dAtBlueSide) {
        Translation2d translation3dAtCurrentAlliance = toCurrentAllianceTranslation(translation3dAtBlueSide.toTranslation2d());
        return isSidePresentedAsRed() ? new Translation3d(translation3dAtCurrentAlliance.getX(), translation3dAtCurrentAlliance.getY(), translation3dAtBlueSide.getZ()) : translation3dAtBlueSide;
    }

    public static Pose2d toCurrentAlliancePose(Pose2d poseAtBlueSide) {
        return new Pose2d(toCurrentAllianceTranslation(poseAtBlueSide.getTranslation()), toCurrentAllianceRotation(poseAtBlueSide.getRotation()));
    }

    public static boolean isSidePresentedAsRed() {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        return alliance.isPresent() && ((DriverStation.Alliance)alliance.get()).equals(Alliance.Red);
    }

    public static Rotation2d getCurrentAllianceDriverStationFacing() {
        return toCurrentAllianceRotation(Rotation2d.kZero);
    }
}