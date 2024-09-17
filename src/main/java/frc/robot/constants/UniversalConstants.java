package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;

public class UniversalConstants {
    private UniversalConstants() {
        throw new UnsupportedOperationException("Cannot construct a constants class");
    }

    public static final double FIELD_WIDTH_METERS = Meters.convertFrom(54, Feet) + Meters.convertFrom(3.25, Inches);
    public static final double FIELD_HEIGHT_METERS = Meters.convertFrom(26, Feet) + Meters.convertFrom(11.25, Inches);

    public static final double FIELD_HALF_WIDTH_METERS = FIELD_WIDTH_METERS / 2.0;
    public static final double FIELD_HALF_HEIGHT_METERS = FIELD_HEIGHT_METERS / 2.0;

    public static final double PI2 = Math.PI * 2.0;

    public static final PositionsContainer RED_POSITIONS = new PositionsContainer("Red");
    public static final PositionsContainer BLUE_POSITIONS = new PositionsContainer("Blue");

    public static boolean isRedSide() {
        if(!DriverStation.isDSAttached()) {
            return false;
        }
        return DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public static PositionsContainer getAlliancePositions() {
        if (isRedSide()) {
            return RED_POSITIONS;
        } else {
            return BLUE_POSITIONS;
        }
    }
}