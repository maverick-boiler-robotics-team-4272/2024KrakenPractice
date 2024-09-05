// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.states.IntakeState;
import frc.robot.subsystems.intake.states.OutakeState;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  private final ShuffleboardTab autoTab;
  private final SendableChooser<Command> autoChooser;
  private final Field2d field;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final Intake intake = new Intake();

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.leftTrigger(0.1).whileTrue(
      new IntakeState(intake, 0.9)
    );

    joystick.rightTrigger(0.1).whileTrue(
      new OutakeState(intake, 0.9)
    );

    joystick.a().whileTrue(drivetrain.applyRequest(
      () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
    ));

    joystick.leftBumper().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on left bumper press
    joystick.b ().onTrue(drivetrain.reset());

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    registerNamedCommands();

    autoChooser = AutoBuilder.buildAutoChooser();

    autoTab = Shuffleboard.getTab("Auto");
    autoTab.add(autoChooser).withSize(2, 1);

    autoChooser.setDefaultOption("TestAuto", new PathPlannerAuto("TestAuto"));
    autoChooser.addOption("Circle Auto", new PathPlannerAuto("Circle Auto"));
    autoChooser.addOption("Three Note Intake", new PathPlannerAuto("Three Note Intake"));
    autoChooser.addOption("LimeLightTest", new PathPlannerAuto("LimeLightTest"));

    field = new Field2d();
    autoTab.add("Field", field).withSize(6, 4);

    // Logging callback for current robot pose
    PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
      // Do whatever you want with the pose here
      if(DriverStation.isAutonomousEnabled())
        field.setRobotPose(pose);
    });

    // Logging callback for target robot pose
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        // Do whatever you want with the pose here
        if(DriverStation.isAutonomousEnabled()) {
          field.getObject("target pose").setPose(pose);
        }
    });

    // Logging callback for the active path, this is sent as a list of poses
    PathPlannerLogging.setLogActivePathCallback((poses) -> {
        // Do whatever you want with the poses here
        field.getObject("path").setPoses(poses);
    });

    drivetrain.setLogCurrentPos((pose) -> {
      field.setRobotPose(pose);
    });

  }

  private void registerNamedCommands() {
      NamedCommands.registerCommand("Intake", new IntakeState(intake, 0.9));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
