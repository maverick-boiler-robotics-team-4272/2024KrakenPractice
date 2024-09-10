package frc.robot.subsystems.shooter.states;

import java.util.function.BooleanSupplier;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.utils.commandUtils.State;

public class SplitShootState extends State<Shooter> {
    double top;
    double bottom;
    BooleanSupplier feed;

    public SplitShootState(Shooter shooter, BooleanSupplier feed, double top, double bottom) {
        super(shooter);

        this.bottom = bottom;
        this.top = top;
        this.feed = feed;
    }

    @Override
    public void initialize() {
        requiredSubsystem.rev(top, bottom);
    }

    @Override
    public void execute() {
        if(feed.getAsBoolean()) {
            requiredSubsystem.feed(0.9);
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.rev(0);
        requiredSubsystem.feed(0);
    }
}
