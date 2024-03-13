// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ArmSubsystem m_arm = new ArmSubsystem();
  public  final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();

  // The driver's controller
  PS4Controller m_driverController = new PS4Controller(OIConstants.kDriverControllerPort);
  XboxController m_operatoController = new XboxController(1);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        //The left stick controls translation of the robot.
        //Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    // Multiply by max speed to map the joystick unitless inputs to actual units.
                    // This will map the [-1, 1] to [max speed backwards, max speed forwards],
                    // converting them to actual units.
                    m_driverController.getLeftY() * DriveConstants.kMaxSpeedMetersPerSecond,
                    -m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond, //changed this to negative
                    -m_driverController.getRawAxis(4)
                        * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    true),
            m_robotDrive));
    m_arm.setDefaultCommand(
      m_arm.trackLimelightCommand());
        
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    final double armSpeed = 0.2;
    
    final JoystickButton xboxButtonMoveArmDown = new JoystickButton(m_operatoController, XboxController.Axis.kLeftY.value);        
    xboxButtonMoveArmDown.whileTrue(m_arm.rotateArmCommand(-armSpeed/2));

    final JoystickButton xboxButtonMoveArmUp = new JoystickButton(m_operatoController, XboxController.Axis.kLeftY.value);      
    xboxButtonMoveArmUp.whileTrue(m_arm.rotateArmCommand(armSpeed));
    //Intake In
    final JoystickButton xboxButtonIntakeIn = new JoystickButton(m_operatoController, XboxController.Axis.kRightY.value);        
    xboxButtonIntakeIn.whileTrue(m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage));
    //Intake Out
    final JoystickButton xboxButtonIntakeOut = new JoystickButton(m_operatoController, XboxController.Axis.kRightY.value);      
    xboxButtonIntakeOut.whileTrue(m_intake.runIntakeCommand(-IntakeConstants.kIntakeVoltage));

    // Speaker1 - Subwooper
    final JoystickButton xboxButtonSp1 = new JoystickButton(m_operatoController, XboxController.Button.kY.value);        
    xboxButtonSp1.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(15)),
      //m_arm.rotateArmCommand(15), 
                           (m_shooter.runShooterCommand(()->0.4))));
    // Speaker2 - preload
    final JoystickButton xboxButtonSp2 = new JoystickButton(m_operatoController, XboxController.Button.kB.value);        
    xboxButtonSp2.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(20)), 
       m_shooter.runShooterCommand(()->0.5)));
    //shooter3 - 13 feet
    final JoystickButton xboxButtonSp3 = new JoystickButton(m_operatoController, XboxController.Button.kX.value);        
    xboxButtonSp3.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(30)), 
      m_shooter.runShooterCommand(()->0.6)));
    //Amp
     final JoystickButton xboxButtonAmp = new JoystickButton(m_operatoController, XboxController.Button.kA.value);        
    xboxButtonAmp.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(90)), 
      m_shooter.runShooterCommand(()->0.2)));

    // Shoot
    final JoystickButton xboxButtonIntake = new JoystickButton(m_operatoController, XboxController.Button.kLeftBumper.value);
    xboxButtonIntake.whileTrue(m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage));

    // Intake
    final JoystickButton xboxButtonShoot = new JoystickButton(m_operatoController, XboxController.Button.kRightBumper.value);
    xboxButtonShoot.whileTrue(new InstantCommand(()->m_arm.setPos(0)).andThen(m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage)));

  }

  public IntakeSubsystem getIntake(){
    return m_intake;
  }
  public DriveSubsystem getDriveSubsystem(){
    return m_robotDrive;
  }
  public ArmSubsystem getArm(){
    return m_arm;
  }
// public Command getAutonomousCommand() {

//   Command shoot1 = new ParallelCommandGroup(m_arm.rotateArmCommand(10),
//                           InstantCommand(()->m_shooter.setSpeed(0.4)));
                        
//   shoot1.until(()->m_arm.getPos() < 11).andThen(m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage));

                  


//   new Commands.sequence()

// }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand2() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(4, 0), new Translation2d(4, 4)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(0)),
            config);
    

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0), 
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the initial pose of the trajectory, run path following
    // command, then stop at the end.
    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
        swerveControllerCommand,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
    
  }
}
