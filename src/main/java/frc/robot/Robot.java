// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.SwerveModule;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.cameraserver.CameraServer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
      m_robotContainer = new RobotContainer();
      m_robotContainer.m_arm.resetArmEncoder();
    m_robotContainer.getDriveSubsystem().zeroHeading();
    CameraServer.startAutomaticCapture();

      //  m_robotContainer.getDriveSubsystem().resetEncoders();

      // m_robotContainer.getDriveSubsystem().resetEncoders();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.getArm().setArmOffset(Constants.ArmConstants.kArmOffset);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
   // m_robotContainer.getDriveSubsystem().resetDriveEncoders();
    // m_robotContainer.getDriveSubsystem().resetOdometry(new Pose2d(0,0,new Rotation2d(0))); // our starting pos isn't really 0 with coords can we do this?


    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}


  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
// m_robotContainer.getDriveSubsystem().resetEncoders();
    m_robotContainer.getDriveSubsystem().zeroHeading();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("FrontLeft", m_robotContainer.getDriveSubsystem().getFrontLeft().getTurningPosition());
    SmartDashboard.putNumber("FrontRight", m_robotContainer.getDriveSubsystem().getFrontRight().getTurningPosition());
    SmartDashboard.putNumber("BackLeft", m_robotContainer.getDriveSubsystem().getBackLeft().getTurningPosition());
    SmartDashboard.putNumber("BackRight", m_robotContainer.getDriveSubsystem().getBackRight().getTurningPosition());
    SmartDashboard.putNumber("FrontLeftPos", m_robotContainer.getDriveSubsystem().getBackRight().getPosition().distanceMeters);
    SmartDashboard.putNumber("FrontRightPos", m_robotContainer.getDriveSubsystem().getBackRight().getPosition().distanceMeters);
    SmartDashboard.putNumber("BackLeftPos", m_robotContainer.getDriveSubsystem().getBackRight().getPosition().distanceMeters);
    SmartDashboard.putNumber("BackRightPos", m_robotContainer.getDriveSubsystem().getBackRight().getPosition().distanceMeters);
    SmartDashboard.putNumber("FrontLeftVel", m_robotContainer.getDriveSubsystem().getBackRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("FrontRightVel", m_robotContainer.getDriveSubsystem().getBackRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BackLeftVel", m_robotContainer.getDriveSubsystem().getBackRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("BackRightVel", m_robotContainer.getDriveSubsystem().getBackRight().getState().speedMetersPerSecond);
    SmartDashboard.putNumber("GyroHeading", m_robotContainer.getDriveSubsystem().getHeading());
    SmartDashboard.putNumber("GyroHeading1", m_robotContainer.getDriveSubsystem().m_gyro.getAngle());
    SmartDashboard.putBoolean("hasNote", m_robotContainer.getIntake().hasNote());
    SmartDashboard.putNumber("2mDist", m_robotContainer.getIntake().getRange());
  }


  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
