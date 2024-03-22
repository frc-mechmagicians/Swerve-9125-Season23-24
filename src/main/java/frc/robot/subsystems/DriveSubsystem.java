// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftDriveEncoderPorts,
          DriveConstants.kFrontLeftTurningEncoderPorts,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftDriveEncoderPorts,
          DriveConstants.kRearLeftTurningEncoderPorts,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightDriveEncoderPorts,
          DriveConstants.kFrontRightTurningEncoderPorts,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightDriveEncoderPorts,
          DriveConstants.kRearRightTurningEncoderPorts,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  //public final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition(),
          });


  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        -xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                    : new ChassisSpeeds(-xSpeed, ySpeed, rot),
                DriveConstants.kDrivePeriod));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  //   SmartDashboard.putNumber("frontLeftAngActual", m_frontLeft.getTurningPosition());
  //   SmartDashboard.putNumber("backLeftAngActual", m_rearLeft.getTurningPosition());
  //   SmartDashboard.putNumber("frontRightAngActual", m_frontRight.getTurningPosition());
  //   SmartDashboard.putNumber("backRightAngActual", m_rearRight.getTurningPosition());
  //   SmartDashboard.putNumber("FrontLeftSpeed", swerveModuleStates[0].speedMetersPerSecond);
  //   SmartDashboard.putNumber("FrontRightSpeed", swerveModuleStates[1].speedMetersPerSecond);
  //   SmartDashboard.putNumber("BackLeftSpeed", swerveModuleStates[2].speedMetersPerSecond);
  //   SmartDashboard.putNumber("BackRightSpeed", swerveModuleStates[3].speedMetersPerSecond);
  //   SmartDashboard.putNumber("FrontLeftAng", swerveModuleStates[0].angle.getRadians());
  //   SmartDashboard.putNumber("FrontRightAng", swerveModuleStates[1].angle.getRadians());
  //   SmartDashboard.putNumber("BackLeftAng", swerveModuleStates[2].angle.getRadians());
  //   SmartDashboard.putNumber("BackRightAng", swerveModuleStates[3].angle.getRadians());
  //   SmartDashboard.putNumber("rot", rot);

    SmartDashboard.putNumber("FLDriveEncoderDistance", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FRDriveEncoderDistance", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("RLDriveEncoderDistance", m_rearLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("RRDriveEncoderDistance", m_rearRight.getPosition().distanceMeters);


   }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public SwerveModule getFrontLeft(){
    return m_frontLeft;
  }
  public SwerveModule getFrontRight(){
    return m_frontRight;
  }
  public SwerveModule getBackLeft(){
    return m_rearLeft;
  }
  public SwerveModule getBackRight(){
    return m_rearRight;
  }
}
