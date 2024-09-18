package frc.robot.subsystems.shooter.states;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.commandUtils.State;

public class ShootState extends State<Shooter> {
    double revSpeed;
    BooleanSupplier feed;

    public ShootState(Shooter shooter, double rev, BooleanSupplier feed) {
        super(shooter);

        revSpeed = rev;
        this.feed = feed;
    }

    @Override
    public void initialize() {
        requiredSubsystem.rev(revSpeed, revSpeed);
    }

    @Override
    public void execute() {
        if(feed.getAsBoolean()) {
            requiredSubsystem.feed(0.75);
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.rev(0, 0);
        requiredSubsystem.feed(0);
    }
}
