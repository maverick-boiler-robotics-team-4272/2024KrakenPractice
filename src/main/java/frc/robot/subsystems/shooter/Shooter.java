package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.NeoBuilder;
import frc.robot.utils.hardware.VortexBuilder;

import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    private CANSparkFlex motor1;
    private CANSparkFlex motor2;
    private CANSparkMax feedMotor;

    public Shooter() {
        motor1 = VortexBuilder.create(SHOOTER_MOTOR_TOP_ID)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kCoast)
            .withInversion(false)
            .build();

        motor2 = VortexBuilder.create(SHOOTER_MOTOR_BOTTOM_ID)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kCoast)
            .withInversion(true)
            .build();

        feedMotor = NeoBuilder.create(FEED_MOTOR_ID)
            .withCurrentLimit(40)
            .withInversion(false)
            .build();
    }

    public void feed(double speed) {
        feedMotor.set(speed);
    }

    public void rev(double topSpeed, double bottomSpeed) {
        motor1.set(topSpeed);
        motor2.set(bottomSpeed);
    }
}
