// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import java.util.ResourceBundle.Control;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import  com.ctre.phoenix6.hardware.CANcoder;


public class SwerveModule {
  
  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final CANcoder m_turningEncoder;



  private final SparkPIDController m_drivePIDController;
  // Using a TrapezoidProfile PIDController to allow for smooth turning
  // private final ProfiledPIDController m_turningPIDController =
  //     new ProfiledPIDController(
  //       ModuleConstants.kPModuleTurningController * 0.5,
  //         0,
  //         0.1,
  //         new TrapezoidProfile.Constraints(
  //             ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
  //             ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared));


  private final PIDController m_turningPIDController = new PIDController(ModuleConstants.kPModuleTurningController * 0.6, 0, 0);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(0.001, 0.05);
  //private final int m_turningEncoderReversed;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel The channel of the drive motor.
   * @param turningMotorChannel The channel of the turning motor.
   * @param driveEncoderChannels The channels of the drive encoder.
   * @param turningEncoderChannels The channels of the turning encoder.
   * @param driveEncoderReversed Whether the drive encoder is reversed.
   * @param turningEncoderReversed Whether the turning encoder is reversed.
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int[] driveEncoderChannels,
      int turningEncoderChannel,
      boolean driveEncoderReversed,
      boolean turningEncoderReversed) {
    m_driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);
    m_turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);
   // m_driveMotor.restoreFactoryDefaults();
    //m_turningMotor.restoreFactoryDefaults();
    //m_driveMotor.setSmartCurrentLimit(Constants.generalMotorSmartLimit);
   // m_turningMotor.setSmartCurrentLimit(Constants.generalMotorSmartLimit);
   m_turningMotor.setIdleMode(IdleMode.kBrake);

    m_driveEncoder = m_driveMotor.getEncoder();

    m_turningEncoder = new CANcoder(turningEncoderChannel);

    m_drivePIDController = m_driveMotor.getPIDController();

    m_driveMotor.setInverted(driveEncoderReversed);
    m_drivePIDController.setP(0.005);
    m_drivePIDController.setI(0);
    m_drivePIDController.setD(0);
    m_drivePIDController.setFF(0.1);
    m_drivePIDController.setOutputRange(-1, 1);
    m_driveEncoder.setPositionConversionFactor(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
    m_driveEncoder.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
    //m_turningPIDController.setTolerance(Math.PI/100);
    //m_turningEncoderReversed = turningEncoderReversed ? 1 : -1;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

    // Set whether drive encoder should be reversed or not
    //m_driveEncoder.setReverseDirection(driveEncoderReversed);
    // m_driveEncoder.setInverted(turningEncoderReversed);
    // Set the distance (in this case, angle) in radians per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(ModuleConstants.kTurningEncoderDistancePerPulse);

    // Set whether turning encoder should be reversed or not
    //m_turningEncoder.setReverseDirection(turningEncoderReversed);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }
  public double getTurningPosition() {
    return -m_turningEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(getTurningPosition()));
  }
  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        m_driveEncoder.getPosition(), new Rotation2d(getTurningPosition()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    var encoderRotation = new Rotation2d(getTurningPosition());

    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
    // direction of travel that can occur when modules change directions. This results in smoother
    // driving.
    state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

    // Calculate the drive output from the drive PID controller.
    /*final double driveOutput =
        m_drivePIDController.calculate(m_driveEncoder.getVelocity(), state.speedMetersPerSecond);
*/
    // Calculate the turning motor output from the turning PID controller.
  final double turnOutput =
        m_turningPIDController.calculate(getTurningPosition(), state.angle.getRadians());
  // final double turnFeedforwardOutput = 
  //       m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    m_turningMotor.setVoltage(turnOutput);
   // m_turningMotor.set(0);  
    //m_driveMotor.set(0.1);
    m_drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
    //m_turningPIDController.setReference(state.angle.getRadians(), ControlType.kPosition);
  }
  

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    m_turningEncoder.setPosition(0);
  }
}

