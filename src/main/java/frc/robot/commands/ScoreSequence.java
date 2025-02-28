package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.CoralHolder;

import static edu.wpi.first.units.Units.Meters;

public class ScoreSequence  extends SequentialCommandGroup {
    public ScoreSequence(Elevator elevator, Arm arm, CoralHolder coralHolder) {
        super.addRequirements(elevator, arm, coralHolder);

        /*
         * The following steps are done in the Autonomous time
         * when the robot are placed in the middle.
         */

        //First step: Shoot the preload note to the speaker
        super.addCommands(Commands.run(() -> {
            arm.moveToPosition(Arm.ArmPosition.SCORE);               //Set the Arm to aim the score Level 2
//            elevator.runSetpoint(Meters.of(0.19));         //Raise to level 2
//            coralHolder.scoreCoral().until(() -> !coralHolder.hasCoral());
        }, arm, elevator, coralHolder).withTimeout(0.8));
    }
}
