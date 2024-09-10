package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.FeedState;

public class IntakeFeedCommand extends ParallelCommandGroup {
    public IntakeFeedCommand(Intake intake, Shooter shooter, double intakeSpeed, double feedSpeed) {
        super(
            new IntakeState(intake, intakeSpeed),
            new FeedState(shooter, feedSpeed)
        );
    }
}
