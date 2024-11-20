package frc.robot.subsystems.drivetrain;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.SubsystemConstants;
import frc.robot.utils.limelight.LimelightHelpers;
import frc.robot.utils.logging.Loggable;

import static frc.robot.constants.AutoConstants.*;
import static frc.robot.constants.SubsystemConstants.LimeLightConstants.BACK_LIMELIGHT;
import static frc.robot.constants.SubsystemConstants.LimeLightConstants.BACK_LIMELIGHT_POSE;
import static frc.robot.constants.SubsystemConstants.LimeLightConstants.FRONT_LIMELIGHT;
import static frc.robot.constants.SubsystemConstants.LimeLightConstants.FRONT_LIMELIGHT_POSE;
import static frc.robot.constants.UniversalConstants.getAlliancePositions;
import static frc.robot.constants.UniversalConstants.isRedSide;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem, Loggable {
    @AutoLog
    public static class DrivetrainInputs {
        public Pose2d estimatedPose; // The Fused Pose of the robot
        public Pose2d notePose;
        public SwerveModuleState moduleStates[]; // The module states of the robot

        // public boolean isPathFinding; // is the robot driving itself
        public boolean fuseVison; // Is the odometry fusing
        public boolean overrideRotation; // Is the path following rotation overriden
        public double distanceTraveled; // How much distance has the robot traveled
        public double distanceTraveledFromTagRead; // How much distance has the robot traveled since last apriltag read
        public double tagDistance;

        public Pose2d limelightPose; //TODO: Move this to limelight periodic
    }

    // Logging inputs
    DrivetrainInputsAutoLogged inputs = new DrivetrainInputsAutoLogged();

    /*
     * This is just some simulation stuff...
     */
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    // Last periodic loop time
    private double lastPeriodicTime = 0.0;

    // Last distance seen a apriltag
    private double lastDistance = 0.0;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;

    // The pose to rotlock to
    private Pose2d rotLockPose;

    // The request for autos driving
    private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        
        initPathPlanner();
        FRONT_LIMELIGHT.configure(FRONT_LIMELIGHT_POSE);

        inputs.estimatedPose = new Pose2d();
        inputs.limelightPose = new Pose2d();
        inputs.notePose = new Pose2d();

        inputs.moduleStates = new SwerveModuleState[4];
        for(int i = 0; i < 4; i++) {
            inputs.moduleStates[i] = getModule(i).getCurrentState();
        }

        inputs.fuseVison = false;
        inputs.overrideRotation = false;
        inputs.distanceTraveledFromTagRead = 0.0;
        inputs.distanceTraveled = 0.0;
        inputs.tagDistance = 0.0;

        rotLockPose = getAlliancePositions().SHOT_POSITION;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Path find to a pose
     * @param target to go to
     */
    public Command pathFind(Pose2d target) {
        return AutoBuilder.pathfindToPose(
            target, 
            new PathConstraints(
                TRANSLATION_MAX,
                TRANSLATION_MAX_A,
                ROTATION_MAX.getRadians(), 
                ROTATION_MAX_A.getRadians()
            )
        );
    }

    /**
     * Reset The robot heading
     */
    public Command reset() {
        return runOnce(
            () -> {
                seedFieldRelative();
                getPigeon2().setYaw(0);
            }
        );
    }

    //Sim stuff
    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public ChassisSpeeds getCurrentChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    /**
     * Set the pose to rotlock to
     * @param pose to set to
     */
    public void setRotLockPose(Pose2d pose) {
        this.rotLockPose = pose;
    }

    public void overrideRotation() {
        inputs.overrideRotation = true;
    }
    
    public void unOverrideRotation() {
        inputs.overrideRotation = false;
    }
    
    private Optional<Rotation2d> getRotationTargetOverride() {
        if(inputs.overrideRotation) {
            return Optional.of(rotLockPose.getTranslation().minus(getState().Pose.getTranslation()).getAngle());
        } else {
            return Optional.empty();
        }
    }

    private void findNote() {
        if(BACK_LIMELIGHT.getTV()) {
            double dy = BACK_LIMELIGHT_POSE.getZ() * Rotation2d.fromRadians(BACK_LIMELIGHT_POSE.getRotation().getY()).plus(Rotation2d.fromDegrees(BACK_LIMELIGHT.getTY())).getTan();
            double dx = dy * Rotation2d.fromDegrees(BACK_LIMELIGHT.getTX()).getTan();

            Pose2d ePose = inputs.estimatedPose;

            double nx = ePose.getX() - dy * ePose.getRotation().getCos() - dx * ePose.getRotation().getSin();
            double ny = ePose.getY() - dy * ePose.getRotation().getSin() + dx * ePose.getRotation().getCos();

            inputs.notePose = new Pose2d(nx, ny, new Rotation2d(0));
        }
    }

    private void initPathPlanner() {
        AutoBuilder.configureHolonomic(
            ()->this.getState().Pose, // Robot pose supplier
            this::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getCurrentChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds)->this.setControl(AutoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(TRANSLATION_P, TRANSLATION_I, TRANSLATION_D), // Translation PID constants
                    new PIDConstants(ROTATION_P, ROTATION_I, ROTATION_D), // Rotation PID constants
                    TRANSLATION_MAX, // Max module speed, in m/s
                    DRIVEBASE_RADIUS, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              return isRedSide();
            },
            this // Reference to this subsystem to set requirements
        );

        PPHolonomicDriveController.setRotationTargetOverride(this::getRotationTargetOverride);
    }

    private void fuseOdometry() {
        FRONT_LIMELIGHT.setRobotOrientation(m_pigeon2.getAngle());

        LimelightHelpers.PoseEstimate limelightMeasurement = SubsystemConstants.LimeLightConstants.FRONT_LIMELIGHT.getBotPoseEstimate();

        if(limelightMeasurement != null) {
            if(limelightMeasurement.tagCount >= 1 && limelightMeasurement.avgTagDist < 4.0) {
                setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                addVisionMeasurement(
                    limelightMeasurement.pose,
                    limelightMeasurement.timestampSeconds
                );

                lastDistance = inputs.distanceTraveled;
                inputs.fuseVison = true;
            } else {
                inputs.fuseVison = false;
            }

            inputs.tagDistance = limelightMeasurement.avgTagDist;
        }

        inputs.limelightPose = limelightMeasurement.pose;
    }

    @Override
    public void log(String subdirectory, String humanReadableName) {
        Logger.processInputs(subdirectory + "/" + humanReadableName, inputs);
    }

    @Override
    public void periodic() {
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            if(isRedSide()) {
                this.setOperatorPerspectiveForward(
                    RedAlliancePerspectiveRotation
                );
            } else {
                this.setOperatorPerspectiveForward(
                    BlueAlliancePerspectiveRotation
                );
            }
        }

        fuseOdometry();

        double deltaTime = Timer.getFPGATimestamp() - lastPeriodicTime;
        lastPeriodicTime = Timer.getFPGATimestamp();

        inputs.distanceTraveled += Math.sqrt(
            Math.pow(getState().speeds.vxMetersPerSecond, 2) +
            Math.pow(getState().speeds.vyMetersPerSecond, 2)
        ) * deltaTime;

        inputs.distanceTraveledFromTagRead = inputs.distanceTraveled - lastDistance;

        inputs.estimatedPose = getState().Pose;

        for(int i = 0; i < 4; i++) {
            inputs.moduleStates[i] = getModule(i).getCurrentState();
        }

        findNote();

        log("Subsystems", "Drivetrain");
    }
}
