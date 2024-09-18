package frc.robot.commands;

import static frc.robot.constants.UniversalConstants.getAlliancePositions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.ShootState;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(CommandSwerveDrivetrain drivetrain, Shooter shooter) {
        super(
            drivetrain.pathFind(getAlliancePositions().SHOT_POSITION).deadlineWith(
                new ShootState(shooter, 1.0, () -> false)
            ),
            new ShootState(shooter, 1.0, () -> true)
        );
    }
}
