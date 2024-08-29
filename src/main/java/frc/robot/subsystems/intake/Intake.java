package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.SubsystemConstants.IntakeConstants.*;

public class Intake extends SubsystemBase {
    public Intake() {
        intakeMotor = VortexBuilder.create(intakeId)
            .withVoltageCompensation(11)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .withPeriodicFramerate(PeriodicFrame.kStatus1, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus2, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus3, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus4, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus5, 500)
            .withPeriodicFramerate(PeriodicFrame.kStatus6, 500)
            .build();
    }

    public void run(double speed) {
        intakeMotor.set(speed);
    }

    CANSparkFlex intakeMotor;
}
