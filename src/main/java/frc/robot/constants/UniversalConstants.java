package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
    public static SendableChooser<String> SIDE_CHOOSER = new SendableChooser<>();

    public static boolean isRedSide() {
        if(SIDE_CHOOSER.getSelected() == "red") {
            return true;
        }

        return  false;
    }

    public static PositionsContainer getAlliancePositions() {
        if (isRedSide()) {
            return RED_POSITIONS;
        } else {
            return BLUE_POSITIONS;
        }
    }
}