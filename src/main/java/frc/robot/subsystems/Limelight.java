package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Limelight {
    public static final double limelightLensHeightInches = 20.0;
    public static final double limelightMountAngleDegrees = 25.0; 
    public static final double goalHeightInches = 60.0;
    public static final double distancefrompivottoLimeLight = 10; 
    public static final double DistanceFromPivotToSpeakerOpening = 68;
    public static final double ArmLength = 23.25;


    private static double armAngleRadians(double d, double h, double AL) {
        double c = Math.hypot(d, h);
        // The variable 'b' is calculated but not used. It can be removed if not needed.
        // double b = Math.sqrt(Math.pow(c, 2) - Math.pow(AL, 2));
        double z = Math.toDegrees(Math.acos(AL / c));
        double y = Math.toDegrees(Math.atan(h / d));
        double armDegreeMovement = 180 - (z + y);
        return armDegreeMovement * Math.PI/180.0;
    }
    public static double readLimelightAngle(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double angleToGoalDegrees =limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return armAngleRadians(distanceFromLimelightToGoalInches + distancefrompivottoLimeLight , DistanceFromPivotToSpeakerOpening, ArmLength);
    }
    public static boolean isAprilTagDetected(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        if (table !=null) {
            return table.getEntry("tv").getBoolean(false);
        }
        return false;
    }
}

