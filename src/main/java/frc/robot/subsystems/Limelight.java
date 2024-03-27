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
    public static final double speakerHight = 80;
    public static final double DistanceFromPivotToSpeakerOpening = speakerHight-pivotHeight;
    public static final double ArmLength = 23.25;
    public static final double shootAngleinDegrees = 90-33;

    public static double solveb(double a, double c, double angle) {
        double coffB = 2 * a * Math.cos(Math.toRadians(angle));
        double coffA = 1;
        double coffC = a*a - c*c;
        double discriminator = coffB*coffB - 4*coffA*coffC;
        if (discriminator > 0) {
            double root1 = (-coffB + Math.sqrt(discriminator))/(2*coffA);
            double root2 = (-coffB - Math.sqrt(discriminator))/(2*coffA);
            if (root1 > 0)
                return root1;
            if (root2 > 0)
                return root2;
        } else if (discriminator ==0) {
            return -coffB/(2*coffA);
        }

        return 0;
    }

    public static double solveAngleB(double a, double b, double c, double angleC) {
       return Math.asin(Math.sin(Math.toRadians(angleC)))*b/c;
    }

    private static double armAngleRadians(double d, double h, double AL) {
        double c = Math.hypot(d, h);

        double b = solveb(AL, c, shootAngleinDegrees);
        if (b <= 0) {
            return -1;
        }
        double angleB = solveAngleB(AL, b, c, shootAngleinDegrees);

        // The variable 'b' is calculated but not used. It can be removed if not needed.
        // double b = Math.sqrt(Math.pow(c, 2) - Math.pow(AL, 2));
        double y = Math.toDegrees(Math.atan(h / d));
        double armDegreeMovement = 180 - (angleB + y);
        return armDegreeMovement * Math.PI/180.0;
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
        SmartDashboard.putNumber("limelightAngle", armAngle*(180/Math.PI));
        
        return armAngle;
    
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

