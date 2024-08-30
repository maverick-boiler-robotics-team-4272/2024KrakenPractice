package frc.robot.subsystems.intake.states;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class OutakeState extends Command {
    Intake intake;
    double speed;

    public OutakeState(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.run(-speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }
}
