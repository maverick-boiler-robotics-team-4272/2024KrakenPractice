package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

import org.littletonrobotics.junction.AutoLog;

public class Intake extends SubsystemBase {
    @AutoLog
    public static class IntakeInputs {
        double speed;
    }

    CANSparkFlex intakeMotor;
    IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    public Intake() {
        intakeMotor = VortexBuilder.create(INTAKE_ID)
            .withVoltageCompensation(11)
            .withCurrentLimit(CURRENT_LIMIT)
            .withIdleMode(IdleMode.kBrake)
            .build();
            try {
                Thread.sleep(30);
            } catch(InterruptedException e) {}

        inputs.speed = 0;
    }

    public void run(double speed) {
        inputs.speed = speed;
        intakeMotor.set(speed);
    }
}
