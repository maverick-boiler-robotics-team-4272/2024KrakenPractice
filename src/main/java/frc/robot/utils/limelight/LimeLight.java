package frc.robot.utils.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;

public class LimeLight {
    private String tableName;
    
    public LimeLight(String tableName) {
        this.tableName = tableName;

        configure(new Pose3d());
    }

    public void configure(Pose3d cameraOffset) {
        LimelightHelpers.setCameraPose_RobotSpace(
            tableName, 
            cameraOffset.getX(), 
            cameraOffset.getY(), 
            cameraOffset.getZ(), 
            cameraOffset.getRotation().getX(), 
            cameraOffset.getRotation().getY(), 
            cameraOffset.getZ()
        );

        LimelightHelpers.setLEDMode_ForceOff(tableName);
    }

    public void turnOnLeds() {
        LimelightHelpers.setLEDMode_ForceOn(tableName);
    }

    public Pose2d getBotPose() {
        return (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue || !DriverStation.isDSAttached()) ? LimelightHelpers.getBotPose2d_wpiBlue(tableName) : LimelightHelpers.getBotPose2d_wpiRed(tableName);
    }

    public LimelightHelpers.PoseEstimate getBotPoseEstimate() {
        return (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue || !DriverStation.isDSAttached()) ? LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName) : LimelightHelpers.getBotPoseEstimate_wpiRed(tableName);
    }
}
