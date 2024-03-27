package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class Limelight {
    public static final double limelightLensHeightInches = 6.95;
    public static final double limelightMountAngleDegrees = 20.0; 
    public static final double aprilTagHeightInches = 53.88;
    public static final double distancefrompivottoLimeLight = 0; 
    public static final double pivotHeight = 11;
    public static final double speakerHight = 79;
    public static final double DistanceFromPivotToSpeakerOpening = speakerHight-pivotHeight;
    public static final double ArmLength = 23.25;
    public static final double shootAngleinDegrees = 58;

    public static double solveAngleA(double a, double c, double angleC) {
        return Math.toDegrees(Math.asin(Math.sin(Math.toRadians(angleC))*(a/c)));
    }

    private static double armAngleRadians(double d, double h, double AL) {
        double c = Math.hypot(d, h);
        double angleA = solveAngleA(AL, c, shootAngleinDegrees);
        double angleB = 180 - angleA - shootAngleinDegrees;
        double y = Math.toDegrees(Math.atan(h / d));    
        double armDegreeMovement = 180 - (angleB + y);
        return Math.toRadians(armDegreeMovement);
    }

    public static double readLimelightAngle(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double angleToGoalDegrees =limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        double distanceFromLimelightToGoalInches = (aprilTagHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        double armAngle = armAngleRadians(distanceFromLimelightToGoalInches + distancefrompivottoLimeLight , 
             DistanceFromPivotToSpeakerOpening, ArmLength);
        SmartDashboard.putNumber("Ty", table.getEntry("ty").getDouble(0));
        SmartDashboard.putBoolean("Detected", isAprilTagDetected());
        SmartDashboard.putBoolean("CheckDetected", table.getEntry("tv").getBoolean(false));
        SmartDashboard.putNumber("DistFromGoal", distanceFromLimelightToGoalInches);
        SmartDashboard.putNumber("limelightAngle", armAngle);
        
        return Math.toRadians(armAngle);
    
    }
    public static boolean isAprilTagDetected(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        if (table.getEntry("tid").getDouble(-1) !=-1) {
            return true;
            // return table.getEntry("tv").getBoolean(false);
        }
        return false;
    }

    
    
}

