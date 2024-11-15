// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.constants.UniversalConstants.*;

import java.util.Set;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeFeedCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.states.ShootState;

import static  frc.robot.constants.SubsystemConstants.*;

public class RobotContainer {
  private ShuffleboardTab autoTab;
  private SendableChooser<Command> autoChooser;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = new Intake();
  private final Shooter shooter = new Shooter();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(TeleopConstants.TRANSLATION_MAX * 0.1).withRotationalDeadband(TeleopConstants.ROTATION_MAX.getRadians() * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

  private final Telemetry logger = new Telemetry(TeleopConstants.TRANSLATION_MAX);

  private void configureBindings() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * TeleopConstants.TRANSLATION_MAX) // Drive forward with negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * TeleopConstants.TRANSLATION_MAX) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * TeleopConstants.ROTATION_MAX.getRadians()) // Drive counterclockwise with negative X (left)
        ));

    joystick.leftTrigger(0.1).whileTrue(
      new IntakeFeedCommand(intake, shooter, 0.9, 0.8)
    );

    joystick.rightTrigger(0.1).whileTrue(
      new IntakeFeedCommand(intake, shooter, -0.9, -0.5)
    );

    joystick.b().onTrue(drivetrain.reset());

    joystick.leftBumper().whileTrue(
      new ShootState(shooter, 0.95, ()->joystick.rightBumper().getAsBoolean())
    );

    joystick.rightBumper().whileTrue(
      new ShootCommand(drivetrain, shooter)
    );

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    registerNamedCommands();
    setupTabs();
  }

  private void setupTabs() {
    autoChooser = new SendableChooser<>();

    autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withSize(2, 1);

    autoChooser.setDefaultOption("TestAuto", new PathPlannerAuto("TestAuto"));
    autoChooser.addOption("Circle Auto", new PathPlannerAuto("Circle Auto"));
    autoChooser.addOption("Three Note Intake", new PathPlannerAuto("Three Note Intake"));
    autoChooser.addOption("LimeLightTest", new PathPlannerAuto("LimeLightTest"));

    SIDE_CHOOSER.addOption("Red", "red");
    SIDE_CHOOSER.setDefaultOption("Blue", "blue");
    autoTab.add("Side", SIDE_CHOOSER);
  }

  private void registerNamedCommands() {
      NamedCommands.registerCommand("Intake", new IntakeState(intake, 0.9));
      NamedCommands.registerCommand("ToAmp", Commands.defer(
        ()->drivetrain.pathFind(getAlliancePositions().AMP_POSE),
        Set.of(drivetrain)
      ));

      NamedCommands.registerCommand("RotLockSpeaker", new InstantCommand(() -> {
        drivetrain.setRotLockPose(getAlliancePositions().SHOT_POSITION);
        drivetrain.overrideRotation();
      }));
      NamedCommands.registerCommand("RotLockAmp", new InstantCommand(() -> {
        drivetrain.setRotLockPose(getAlliancePositions().AMP_POSE);
        drivetrain.overrideRotation();
      }));
      NamedCommands.registerCommand("UnLock", new InstantCommand(drivetrain::unOverrideRotation));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
