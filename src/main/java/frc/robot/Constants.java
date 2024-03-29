
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final int generalMotorSmartLimit = 35;
  public static final int intakeMotorSmartLimit = 25;

  //Note Locations
  public static final double[] subwooferLocation = {2.6578, 0.9176+0.25};
  public static final double[] note1Location = {1.21,2.887};
  public static final double[] note2Location = {2.6578,2.887};
  public static final double[] note3Location = {4.1056,2.887};
  public static final double[] note4Location = {0.753,8.261};
  public static final double[] note5Location = {2.429,8.261};
  public static final double[] note6Location = {4.106,8.261};
  public static final double[] note7Location = {5.782,8.261};
  public static final double[] note8Location = {7.458,8.261};

  public static final class IntakeConstants {
    public static final int kIntakeMotorPort = 17; //spark flex 
    public static final int kIntakeVoltage = 4; //chg to 4
    public static final int kUltrasonicSensorPort = 0;
  }

  public static final class ArmConstants {
    public static final int kEncoderPort1 = 0;
    public static final int kEncoderPort2 = 1;
    public static final int kLeftPivotMotorPort = 9;
    public static final int kRightPivotMotorPort = 10;
    public static final double kArmEncoderDistancePerPulse = 360.0/2048.0; // 90.0/2400; //degrees
    public static final double kP = 0.35; //0.35
    public static final double kI = 0;//0
    public static final double kD = 0.0;
    public static final double ks = 0;
    public static final double kg = 0.4; //.5
    public static final double kv = 4; // 4.68;//0.08
    public static final double ka = 0.03;
    public static final double kFeedforwardVelocity = 0.1;
    /* Reca.lc
     * Motor: 2
     * Ratio: 240
     * Efficieny: 100%
     * Curr: 40
     * CoM dis: 22
     * arm mass: 27 lbs
     * start ang 90 deg
     * end ang 180 deg
     * iter limit 10000
     */

     public static final double kArmAnglePickNote = 0;
     public static final double kArmAngleAmp = 90;
     public static final double kArmAngleSubwoofer = 20.5;
     public static final double kArmAnglePreload = 32;
     public static final double kArmAngleLongRange = 32;
     
     public static final double kArmOffset = -90;
  }

  public static final class ShooterConstants {
    public static final int kLeftShooterMotorPort = 15; //spark max
    public static final int kRightShooterMotorPort = 16; // sparkflex
         
    public static final double kShooterSpeedAmp = 0.25;
    public static final double kShooterSpeedSubwoofer = 0.7;
    public static final double kShooterSpeedPreload = 0.8;
    public static final double kShooterSpeedLongRange = 0.8;

  }

  

  public static final class ClimberConstants {
    public static final int kClimberLeftMotorId = 18; 
    public static final int kClimberRightMotorId = 19; 
    public static final double kClimberP = 0.001;
    public static final double kClimberI = 0.0;
    public static final double kClimberD = 0.0;
    public static final double kClimberMinOutput = -0.5;
    public static final double kClimberMaxOutput = 0.5;
    public static final double kClimberGearRatio = 1.0 / 16.0;
  }

  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 8;
    public static final int kRearLeftDriveMotorPort = 6;
    public static final int kFrontRightDriveMotorPort = 2;
    public static final int kRearRightDriveMotorPort = 3;

    public static final int kFrontLeftTurningMotorPort = 7;
    public static final int kRearLeftTurningMotorPort = 5;
    public static final int kFrontRightTurningMotorPort = 1;
    public static final int kRearRightTurningMotorPort = 4;

    public static final int kFrontLeftTurningEncoderPorts = 12;
    public static final int kRearLeftTurningEncoderPorts = 11;
    public static final int kFrontRightTurningEncoderPorts = 13;
    public static final int kRearRightTurningEncoderPorts = 14;

    public static final boolean kFrontLeftTurningEncoderReversed = false;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    public static final int[] kFrontLeftDriveEncoderPorts = new int[] {8, 9};
    public static final int[] kRearLeftDriveEncoderPorts = new int[] {10, 11};
    public static final int[] kFrontRightDriveEncoderPorts = new int[] {12, 13};
    public static final int[] kRearRightDriveEncoderPorts = new int[] {14, 15};

    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = true;
    public static final boolean kFrontRightDriveEncoderReversed = true;
    public static final boolean kRearRightDriveEncoderReversed = true;

    // If you call DriveSubsystem.drive() with a different period make sure to update this.
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    public static final double kTrackWidth = 0.567;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.567;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2*Math.PI;

    //public static final int kEncoderCPR = 1024;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts

       (kWheelDiameterMeters* Math.PI) / 6.75; // 6.75; I think this is what we changed

     //public static final double kTurningEncoderDistancePerPulse =
    //     // Assumes the encoders are on a 1:1 reduction with the module shaft.
     //    (2 * Math.PI) / (double) kEncoderCPR;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kAuotSpeed = 1;
    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
