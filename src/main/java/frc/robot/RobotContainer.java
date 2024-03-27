// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
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
  public final Limelight m_Limelight = new Limelight();
  public final AutoRoutines m_auto = new AutoRoutines(m_robotDrive, m_arm, m_intake, m_shooter); 
  public  double m_robotY = 0.9176+0.25;
  public  double m_robotX = 2.6578;

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
                    -(m_driverController.getLeftY() * m_driverController.getLeftY() * Math.signum(m_driverController.getLeftY()))* DriveConstants.kMaxSpeedMetersPerSecond ,
                    -(m_driverController.getLeftX() * m_driverController.getLeftX() * Math.signum(m_driverController.getLeftX())) * DriveConstants.kMaxSpeedMetersPerSecond, //changed this to negative
                    -m_driverController.getRawAxis(4)/1.5
                        * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
                    true),
            m_robotDrive));

    m_arm.setDefaultCommand(m_arm.trackLimelightCommand());
    // m_intake.setDefaultCommand(new RunCommand(()->
    //     m_intake.setSpeed(-m_operatoController.getRightY()*0.7), m_intake));
    // m_shooter.setDefaultCommand(new RunCommand(()->
    //     m_shooter.setSpeed(-m_operatoController.getLeftY()*0.7), m_shooter)); 
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Speaker1 - Subwoofer
    final JoystickButton xboxButtonSp1 = new JoystickButton(m_operatoController, XboxController.Button.kA.value);        
    xboxButtonSp1.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(ArmConstants.kArmAngleSubwoofer)),
                           (m_shooter.runShooterCommand(()->ShooterConstants.kShooterSpeedSubwoofer))));
    // Speaker2 - preload
    final JoystickButton xboxButtonSp2 = new JoystickButton(m_operatoController, XboxController.Button.kB.value);        
    xboxButtonSp2.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(ArmConstants.kArmAnglePreload)), 
       m_shooter.runShooterCommand(()->ShooterConstants.kShooterSpeedPreload)));
    //shooter3 - 13 feet
    final JoystickButton xboxButtonSp3 = new JoystickButton(m_operatoController, XboxController.Button.kX.value);        
    xboxButtonSp3.whileTrue(m_shooter.runShooterCommand(()->ShooterConstants.kShooterSpeedLongRange));
    xboxButtonSp3.onTrue(new InstantCommand(()->m_arm.enableTracking(true)));
    xboxButtonSp3.onFalse(new InstantCommand(()->m_arm.enableTracking(false)));

    //Amp
     final JoystickButton xboxButtonAmp = new JoystickButton(m_operatoController, XboxController.Button.kY.value);        
    xboxButtonAmp.whileTrue(new ParallelCommandGroup(new InstantCommand(()->m_arm.setPos(ArmConstants.kArmAngleAmp)), 
      m_shooter.runShooterCommand(()->ShooterConstants.kShooterSpeedAmp)));

    // Shoot
    final JoystickButton xboxButtonIntake = new JoystickButton(m_operatoController, XboxController.Button.kLeftBumper.value);
    xboxButtonIntake.whileTrue(m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage));

    // PickNote
    final JoystickButton xboxButtonPick = new JoystickButton(m_operatoController, XboxController.Button.kRightBumper.value);
    xboxButtonPick.whileTrue(
      new InstantCommand(()->m_arm.setPos(ArmConstants.kArmAnglePickNote))
      .andThen(m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage))
      .andThen(new InstantCommand(() -> {
        if (m_intake.hasNote()) m_arm.setPos(ArmConstants.kArmAngleSubwoofer);
      })));

    // xboxButtonPick.onFalse(m_intake.runIntakeCommand(-IntakeConstants.kIntakeVoltage).unless(m_intake::hasNote).withTimeout(0.15));

    final JoystickButton PS4GyroReset = new JoystickButton(m_driverController, PS4Controller.Button.kCross.value);
    PS4GyroReset.onTrue(new InstantCommand(()->m_robotDrive.zeroHeading()));
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
  public Limelight getLimelight(){
    return m_Limelight;
  }
  public Command getAutonomousCommand() {
    return m_auto.AutoRoutineFourPiece();
  }


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
            .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(-2, 0, new Rotation2d(0))),
            // End 3 meters straight ahead of where we started, facing forward
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


  public Command getAutonomousCommand3() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    //               var thetaController =
    //     new ProfiledPIDController(
    //         AutoConstants.kPThetaController*0.2, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    var traj1_1 = auto3_1(config);
        SwerveControllerCommand autoPick1 = setSwerveCommand(traj1_1);
      // new SwerveControllerCommand(
      //     traj1,
      //     m_robotDrive::getPose, // Functional interface to feed supplier
      //     DriveConstants.kDriveKinematics,

      //     // Position controllers
      //     new PIDController(AutoConstants.kPXController, 0, 0), 
      //     new PIDController(AutoConstants.kPYController, 0, 0),
      //     thetaController,
      //     m_robotDrive::setModuleStates,
      //     m_robotDrive);
    var traj2_1 = auto3_2(config);
        SwerveControllerCommand autoPick2 = setSwerveCommand(traj2_1);
    var traj3_1 = auto3_3(config);
        SwerveControllerCommand autoPick3 = setSwerveCommand(traj3_1);




    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(traj1_1.getInitialPose())),
        autoPick2,
        //autoPick1,
        //autoPick3,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  
  }


  public Command getAutonomous3AwayFromSubwoofer(){
    TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond-2,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared-2)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    var traj1 = auto3Far_1(config);
        SwerveControllerCommand autoPick1 = setSwerveCommand(traj1);
    var traj1_1 = auto3Far_1_Shoot(config);
        SwerveControllerCommand autoShootPos1 = setSwerveCommand(traj1_1);
    var traj2 = auto3Far_2(config);
        SwerveControllerCommand autoPick2 = setSwerveCommand(traj2);
    var traj2_1 = auto3Far_2_Shoot(config);
        SwerveControllerCommand autoShootPos2= setSwerveCommand(traj2_1);

    return Commands.sequence(
        new InstantCommand(() -> m_robotDrive.resetOdometry(traj1.getInitialPose())),
         autoPick1,
         autoShootPos1,
        autoPick2,
        autoShootPos2,
        new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  

  }
  


  public SwerveControllerCommand setSwerveCommand(Trajectory traj){
    
    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController*0.2, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);  
    SwerveControllerCommand autoTraj =
      new SwerveControllerCommand(
          traj,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,

          // Position controllers
          new PIDController(AutoConstants.kPXController*2, 0, 0), 
          new PIDController(AutoConstants.kPYController*2, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);
      return autoTraj;
  }


  public Trajectory auto3Far_1(TrajectoryConfig config){
            Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.2, 0),new Translation2d(1.4, -3.35)),
            // End 3 meters straight ahead of wh`ere we started, facing forward
            new Pose2d(8.1, -3.35, new Rotation2d(0)),
            config);
        return exampleTrajectory;
  }

  public Trajectory auto3Far_1_Shoot(TrajectoryConfig config){
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(new Pose2d(8.1,-3.35, new Rotation2d(0)),
        List.of(new Translation2d(2, -3.34),new Translation2d(1, 0)),
        new Pose2d(0.4, 0, new Rotation2d(0)), config);
        return exampleTrajectory;
  }

    public Trajectory auto3Far_2(TrajectoryConfig config){
            Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0.4, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.8, -2), new Translation2d(4, -2.5),new Translation2d(7.3, -2.5)),
            // End 3 meters straight ahead of wh`ere we started, facing forward
            new Pose2d(8.1, -1.63, new Rotation2d(0)),
            config);
        return exampleTrajectory;
  }

    public Trajectory auto3Far_2_Shoot(TrajectoryConfig config){
            Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(8.1, -1.63, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(7.3, -2.5), new Translation2d(3, -2.5), new Translation2d(0.8, -2)),
            // End 3 meters straight ahead of wh`ere we started, facing forward
            new Pose2d(0.4, 0, new Rotation2d(0)),
            config);
        return exampleTrajectory;
  }


  public Pose2d notePose(double loc[], double xOffset, double yOffset, double angleInDegrees) {
    return new Pose2d(-(loc[0]-m_robotX+xOffset), -(loc[1]-m_robotY+yOffset), new Rotation2d(Math.PI/180*angleInDegrees));

  }
  public Trajectory auto3_1(TrajectoryConfig config){
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
          List.of(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            notePose(Constants.note2Location, 0,0,0)),
            config);
        return exampleTrajectory;
  }




    public Trajectory auto3_2(TrajectoryConfig config){
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(
              notePose(Constants.note2Location, 0,0,0),
              notePose(Constants.note1Location, 0.5,-0.5,-45),
              notePose(Constants.note1Location, 0,0,-45)),

              // new Pose2d(Constants.note2Location[1], Constants.note2Location[0], new Rotation2d(0)),
              // new Pose2d(Constants.note1Location[1]+0.5, Constants.note1Location[0]-0.5, new Rotation2d(Math.PI/4)),
              // new Pose2d(Constants.note1Location[1], Constants.note1Location[0], new Rotation2d(Math.PI/4))),
            // Pass through these two interior waypoints, making an 's' curve path),
            // End 3 meters straight ahead of where we started, facing forward
            config);
        return exampleTrajectory;
  }

  public Trajectory auto3_3(TrajectoryConfig config){
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
              notePose(Constants.note1Location, 0, 0, -45),
              notePose(Constants.note3Location, -0.5, -0.5, 45),
              notePose(Constants.note3Location, 0, 0, 45)),

            // new Pose2d(Constants.note1Location[1], Constants.note1Location[0], new Rotation2d(Math.PI/4)),
            // new Pose2d(Constants.note3Location[1]+0.5, Constants.note3Location[0]+0.5, new Rotation2d(-Math.PI/4)),
            // new Pose2d(Constants.note3Location[1], Constants.note3Location[0], new Rotation2d(-Math.PI/4))),
            // End 3 meters straight ahead of where we started, facing forward
            config);
        return exampleTrajectory;
  }




  // public Command getAutonomousCommand3_1() {
  //   // Create config for trajectory
  //   TrajectoryConfig config =
  //       new TrajectoryConfig(
  //               AutoConstants.kMaxSpeedMetersPerSecond-2.5,
  //               AutoConstants.kMaxAccelerationMetersPerSecondSquared-2.5)
  //           // Add kinematics to ensure max speed is actually obeyed
  //           .setKinematics(DriveConstants.kDriveKinematics);

  //       Trajectory exampleTrajectory =
  //       TrajectoryGenerator.generateTrajectory(
  //           // Start at the origin facing the +X direction
  //           new Pose2d(0, 0, new Rotation2d(0)),
  //           // Pass through these two interior waypoints, making an 's' curve path
  //           List.of(),
  //           // End 3 meters straight ahead of where we started, facing forward
  //           new Pose2d(.5, 0.5, new Rotation2d(Math.PI/4)),
  //           config);
    

      
  //     var thetaController =
  //       new ProfiledPIDController(
  //           AutoConstants.kPThetaController*0.2, 0, 0, AutoConstants.kThetaControllerConstraints);
  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //     SwerveControllerCommand swerveControllerCommand =
  //     new SwerveControllerCommand(
  //         exampleTrajectory,
  //         m_robotDrive::getPose, // Functional interface to feed supplier
  //         DriveConstants.kDriveKinematics,

  //         // Position controllers
  //         new PIDController(AutoConstants.kPXController, 0, 0), 
  //         new PIDController(AutoConstants.kPYController, 0, 0),
  //         thetaController,
  //         m_robotDrive::setModuleStates,
  //         m_robotDrive);

  //  // m_robotDrive.resetEncoders();
  //   return Commands.sequence(
  //       new InstantCommand(() -> m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose())),
  //       swerveControllerCommand,
  //       new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false)));
  // }
  
}