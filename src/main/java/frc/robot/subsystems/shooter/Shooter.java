package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.hardware.Neo;
import frc.robot.utils.hardware.NeoBuilder;
import frc.robot.utils.hardware.VortexBuilder;
import frc.robot.utils.logging.Loggable;

import static frc.robot.constants.SubsystemConstants.ShooterConstants.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase implements Loggable {
    @AutoLog
    public static class ShooterInputs {
        double feedPercentage;
        double topShooterPercentage;
        double bottomShooterPercentage;
    }

    private CANSparkFlex motor1;
    private CANSparkFlex motor2;
    private Neo feedMotor;
    private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    public Shooter() {
        motor1 = VortexBuilder.create(SHOOTER_MOTOR_TOP_ID)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(true)
            .build();

        motor2 = VortexBuilder.create(SHOOTER_MOTOR_BOTTOM_ID)
            .withCurrentLimit(40)
            .withIdleMode(IdleMode.kBrake)
            .withInversion(false)
            .build();

        feedMotor = NeoBuilder.create(FEED_MOTOR_ID)
            .withCurrentLimit(40)
            .withInversion(false)
            .build();

        inputs.bottomShooterPercentage = 0.0;
        inputs.topShooterPercentage = 0.0;
        inputs.feedPercentage = 0.0;
    }

    public void feed(double speed) {
        feedMotor.set(speed);
    }

    public void rev(double topSpeed, double bottomSpeed) {
        motor1.set(topSpeed);
        motor2.set(bottomSpeed);
    }

    public void rev(double revSpeed) {
        motor1.set(revSpeed);
        motor2.set(revSpeed);
    }

    @Override
    public void periodic() {
        log("Subsystems", "Shooter");
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        feedMotor.log(subdirectory, humanReadableName);
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }
}
