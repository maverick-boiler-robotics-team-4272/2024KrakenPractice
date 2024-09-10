package frc.robot.subsystems.shooter.states;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.commandUtils.State;

public class FeedState extends State<Shooter> {
    double feedSpeed;

    public FeedState(Shooter shooter, double feedSpeed) {
        super(shooter);

        this.feedSpeed = feedSpeed;
    }

    @Override
    public void initialize() {
        requiredSubsystem.feed(feedSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.feed(0.0);
    }
}
