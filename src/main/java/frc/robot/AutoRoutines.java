package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRoutines {
    private DriveSubsystem m_drive;
    private ArmSubsystem m_arm;
    private IntakeSubsystem m_intake; 
    private ShooterSubsystem m_shooter;
    public  double m_robotY = 0.9176+0.25;
    public  double m_robotX = 2.6578;

    public AutoRoutines(DriveSubsystem drive, ArmSubsystem arm, IntakeSubsystem intake, ShooterSubsystem shooter)
    {
        this.m_drive = drive;
        this.m_arm = arm;
        this.m_intake = intake;
        this.m_shooter = shooter;
    }

    public Command positionArmAndShoot(double armAngle) {
        return Commands.sequence(m_arm.rotateArmCommand(armAngle),
                          m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5));

    }

    public Command autoInit(double shooterSpeed, double armAngle){
        return Commands.sequence(
            m_shooter.runOnce(()->m_shooter.setSpeed(shooterSpeed)), // Start shooter moter
            m_arm.resetCommand(),
            m_arm.rotateArmCommand(armAngle));

    }
    public Command AutoRoutineOnePiece() {
        return Commands.sequence(
            this.autoInit(ShooterConstants.kShooterSpeedSubwoofer, ArmConstants.kArmAngleSubwoofer),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(1.5) // Shoot
        );
    }

    public Command AutoRoutineTwoPiece() {
        return Commands.sequence(
            AutoRoutineOnePiece(), 
            m_shooter.runOnce(()->m_shooter.setSpeed(0)), // Increase shooter speed
      
            // Drive back until note is picked
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(-0.6, 0, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(2),

            // pull back the note before starting shooter
            m_intake.runIntakeCommand(-0.1).unless(m_intake::hasNote).withTimeout(0.15),
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedSubwoofer)), // Increase shooter speed

            // Rotate arm to shooting posiiton while moving forard
            Commands.parallel(
                m_arm.rotateArmCommand(ArmConstants.kArmAngleSubwoofer), // Rotate arm
                m_drive.run(()->m_drive.drive(0.6, 0, 0, false)) // drive forward
            ).withTimeout(2.1),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(1),// Shoot
            m_shooter.runOnce(()->m_shooter.setSpeed(0)) // Increase shooter speed7
        );
    }

    public Command AutoRoutineThreePiece() {
        return Commands.sequence(
            AutoRoutineTwoPiece(),

            // Drive left for third  note
            m_drive.run(()->m_drive.drive(0, 0.2, 0, false)).alongWith(m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote)),

                
            // Drive back until note is picked
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(-0.2, 0, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(1.5),

            // pull back the note before starting shooter
            m_intake.runIntakeCommand(-0.1).unless(m_intake::hasNote).withTimeout(0.15),
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedPreload)), // Increase shooter speed

            // Rotate arm to shooting posiiton while moving forard
            Commands.deadline(
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePreload), // Rotate arm
                m_drive.run(()->m_drive.drive(0.2, 0, 0, false)) // drive forward
            ),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5)// Shoot
        );
    }

    public Pose2d notePose(double loc[], double xOffset, double yOffset, double angleInDegrees) {
        return new Pose2d(-(loc[1]+yOffset), (loc[0]+xOffset), new Rotation2d(angleInDegrees*Math.PI/180));
    }

    public SwerveControllerCommand setSwerveCommand(Trajectory traj){
    
        var thetaController =  new ProfiledPIDController(
            AutoConstants.kPThetaController*0.2, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);  
        SwerveControllerCommand autoTraj = new SwerveControllerCommand(
            traj,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController*2, 0, 0), 
            new PIDController(AutoConstants.kPYController*2, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);
      return autoTraj;
    }

    public SwerveControllerCommand newSwerveControllerCommand(double loc[], double xOffset, double yOffset, double angleInDegrees){

         // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond - 2,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared-2)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);
    
        Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            List.of(
                m_drive.getPose(),
                notePose(loc, xOffset,yOffset,angleInDegrees),
                notePose(loc, 0,0,angleInDegrees)),
            config);
        return setSwerveCommand(trajectory);
  }
  public Command getAutonomousSubwoofer213() {
  
    SwerveControllerCommand autoPickNote2 = newSwerveControllerCommand(Constants.note2Location, 0, -0.5, 0);
    SwerveControllerCommand autoPickNote1 = newSwerveControllerCommand(Constants.note1Location, 0.5, -0.5, 45);
    SwerveControllerCommand autoPickNote3 = newSwerveControllerCommand(Constants.note3Location, -0.5, -0.5, -45);

    return Commands.sequence(
        new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(m_robotY,m_robotX,new Rotation2d(0)))),
        autoPickNote2,
        // autoPickNote1,
        // autoPickNote3,
        new InstantCommand(() -> m_drive.drive(0, 0, 0, false)));
  
  }


    public Command getAutonomousSubwooferFar() {
  
    SwerveControllerCommand autoPickNote8 = newSwerveControllerCommand(Constants.note8Location, 0, -6, 0);
    SwerveControllerCommand autoPickNote7 = newSwerveControllerCommand(Constants.note7Location, 1.5, -4, 45);

    return Commands.sequence(
        new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(m_robotY,m_robotX,new Rotation2d(0)))),
        autoPickNote8,
        autoPickNote7,
        // autoPickNote3,
        new InstantCommand(() -> m_drive.drive(0, 0, 0, false)));
  
  }

}
