package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoAlignment;
import frc.robot.constants.DriveControlLoops;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltags.AprilTagVision;
import frc.robot.utils.FieldMirroringUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.OptionalInt;
import java.util.function.Supplier;

import static frc.robot.constants.ReefConstants.*;

public class ReefAlignment {
    private static final Translation2d REEF_CENTER_BLUE = new Translation2d(4.5, 4);

    public record BranchTarget(
            Rotation2d facing,
            Translation2d roughApproachPosition,
            Translation2d preciseAlignmentPosition,
            int tagId,
            boolean rightSide) {

        public Pose2d roughApproachPose() {
            return new Pose2d(roughApproachPosition, facing);
        }

        public Pose2d preciseAlignmentPose() {
            return new Pose2d(preciseAlignmentPosition, facing);
        }

        public AutoAlignment.AutoAlignmentTarget autoAlignmentTarget() {
            return new AutoAlignment.AutoAlignmentTarget(
                    roughApproachPose(),
                    preciseAlignmentPose(),
                    facing(),
                    Optional.of(FieldMirroringUtils.toCurrentAllianceTranslation(REEF_CENTER_BLUE)),
                    OptionalInt.of(tagId),
                    rightSide ? 0 : 1,
                    rightSide ? 2 : 3);
        }
    }

    // 0 to 5
    private static int selectedReefPartId = 0;
    private static final Subsystem lock = new Subsystem() {};

    public static BranchTarget getReefAlignmentTarget(boolean rightSide) {
        int branchIndex = getBranchIndexFromReefPartId(rightSide);
        return FieldMirroringUtils.isSidePresentedAsRed()
                ? REEF_ALIGNMENT_POSITIONS_RED[branchIndex]
                : REEF_ALIGNMENT_POSITIONS_BLUE[branchIndex];
    }

    private static int getBranchIndexFromReefPartId(boolean rightSide) {
        int branchIndex = selectedReefPartId * 2;
        boolean isUpperSide = // selectedReefPartId == 2 || selectedReefPartId == 3 || selectedReefPartId == 4;
                false;
        if (rightSide ^ isUpperSide) branchIndex++;
        return branchIndex;
    }

    public static boolean[] displaySelectedBranch() {
        boolean[] reef = new boolean[12];
        Arrays.fill(reef, false);
        switch (selectedSide) {
            case LEFT -> reef[getBranchIndexFromReefPartId(false)] = true;
            case RIGHT -> reef[getBranchIndexFromReefPartId(true)] = true;
            case NOT_SELECTED -> reef[getBranchIndexFromReefPartId(false)] =
                    reef[getBranchIndexFromReefPartId(true)] = true;
        }
        return reef;
    }

    public static void nextTarget() {
        if (++selectedReefPartId >= 6) selectedReefPartId = 0;
    }

    public static void previousTarget() {
        if (--selectedReefPartId < 0) selectedReefPartId = 5;
    }

    public static void selectReefPart(int reefPartId) {
        selectedReefPartId = reefPartId;
    }

    public static Command selectReefPartButton(int reefPartId) {
        return Commands.runOnce(() -> selectReefPart(reefPartId), lock);
    }

    public static Command nextTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::nextTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command previousTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::previousTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static void lefterTarget() {
        if (selectedReefPartId == 0 || selectedReefPartId == 1 || selectedReefPartId == 5) previousTarget();
        else nextTarget();
    }

    public static void righterTarget() {
        if (selectedReefPartId == 0 || selectedReefPartId == 1 || selectedReefPartId == 5) nextTarget();
        else previousTarget();
    }

    public static Command lefterTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::lefterTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }

    public static Command righterTargetButton(double debugTime) {
        return Commands.runOnce(ReefAlignment::righterTarget, lock)
                .andThen(Commands.waitSeconds(debugTime))
                .repeatedly();
    }


    public static Command alignmentToBranch(
            CommandSwerveDrivetrain drive,
            AprilTagVision aprilTagVision,
            boolean rightSide,
            Supplier<Command> toRunAtPreciseAlignment) {
        return Commands.deferredProxy(() -> AutoAlignment.pathFindAndAutoAlign(
                        drive,
                        aprilTagVision,
                        ReefAlignment.getReefAlignmentTarget(rightSide).autoAlignmentTarget(),
                        toRunAtPreciseAlignment.get(),
                        DriveControlLoops.REEF_ALIGNMENT_CONFIG))
                .beforeStarting(() -> selectedSide = rightSide ? SelectedSide.RIGHT : SelectedSide.LEFT)
                .finallyDo(() -> selectedSide = SelectedSide.NOT_SELECTED)
                .finallyDo((interrupted) -> {
                    if (!interrupted) alignmentComplete().schedule();
                });
    }

    private enum SelectedSide {
        LEFT,
        RIGHT,
        NOT_SELECTED
    }

    private static SelectedSide selectedSide = SelectedSide.NOT_SELECTED;

    public static void updateDashboard() {
        Logger.recordOutput("Reef/SelectedBranch", ReefAlignment.displaySelectedBranch());
    }

    private static Command roughAlignmentLight() {
        return Commands.none();
    }

    private static Command preciseAlignmentLight() {
        return Commands.none();
    }

    private static Command alignmentComplete() {
        return Commands.none();
    }
}
