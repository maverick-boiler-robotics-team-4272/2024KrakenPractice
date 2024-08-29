package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeState extends Command {
    Intake intake;
    DoubleSupplier speed;

    public IntakeState(Intake intake, DoubleSupplier speed) {
        this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        intake.run(speed.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        intake.run(0);
    }
}
