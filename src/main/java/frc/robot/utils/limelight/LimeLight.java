package frc.robot.utils.limelight;

import static frc.robot.constants.UniversalConstants.isRedSide;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

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

        LimelightHelpers.setPipelineIndex(tableName, 0);
        LimelightHelpers.setLEDMode_PipelineControl(tableName);
        LimelightHelpers.setLEDMode_ForceOff(tableName);
    }

    public void turnOnLeds() {
        LimelightHelpers.setLEDMode_ForceOn(tableName);
    }

    public void setRobotOrientation(double yaw) {
        LimelightHelpers.SetRobotOrientation(tableName, yaw, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    public Pose2d getBotPose() {
        return (!isRedSide()) ? LimelightHelpers.getBotPose2d_wpiBlue(tableName) : LimelightHelpers.getBotPose2d_wpiRed(tableName);
    }

    public LimelightHelpers.PoseEstimate getBotPoseEstimate() {
        return (!isRedSide()) ? LimelightHelpers.getBotPoseEstimate_wpiBlue(tableName) : LimelightHelpers.getBotPoseEstimate_wpiRed(tableName);
    }
}
