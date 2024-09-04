package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utils.limelight.LimeLight;

public class SubsystemConstants {
    public static class IntakeConstants {
        public static final int INTAKE_ID = 15;
        public static final boolean INVERTED = false;
        public static final int CURRENT_LIMIT = 80;
    }

    public static class LimeLightConstants {
        public static Pose3d FRONT_LIMELIGHT_POSE = new Pose3d(
            0.26545,
            0.167063,
            0.223797,
            new Rotation3d(
                0, 
                0.3507, 
                Rotation2d.fromDegrees(10).getRadians()
            )
        );
        public static LimeLight FRONT_LIMELIGHT = new LimeLight("limelight-back");
    }
}
