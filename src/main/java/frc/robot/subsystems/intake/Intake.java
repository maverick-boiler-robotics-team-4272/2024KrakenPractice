package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {
    public Intake() {
        intakeMotor = new CANSparkFlex(intakeId, MotorType.kBrushless);
        intakeMotor.setInverted(inverted);

        intakeMotor.clearFaults();
    }

    public void run(double speed) {
        intakeMotor.set(0.5);
    }

    CANSparkFlex intakeMotor;
}
