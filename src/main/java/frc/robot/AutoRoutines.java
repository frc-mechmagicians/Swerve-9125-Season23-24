package frc.robot;

import java.util.ArrayList;
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
    public double angleForShooting = 30.168;


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
            new InstantCommand(()->m_arm.setArmOffset(Constants.ArmConstants.kArmOffset)),
            m_shooter.runOnce(()->m_shooter.setSpeed(shooterSpeed)), // Start shooter moter
            m_arm.rotateArmCommand(armAngle)).withTimeout(1);
    }
    public Command AutoRoutineOnePiece() {
        return Commands.sequence(
            this.autoInit(ShooterConstants.kShooterSpeedSubwoofer, ArmConstants.kArmAngleSubwoofer),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5), //shoot
            m_shooter.runOnce(()->m_shooter.setSpeed(0))  
        );
        
    }

    public Command AutoRoutineTwoPiece2() {
        return Commands.sequence(
            new InstantCommand(()->m_arm.setArmOffset(0)),
            m_arm.rotateArmCommand(15),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(5),
            m_arm.rotateArmCommand(30),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(5)
            //m_arm.rotateArmCommand(50),
            //m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(5),
            //m_arm.rotateArmCommand(30),
           // m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(5),
            //m_arm.rotateArmCommand(10)

        );
    }
    public Command AutoRoutineTwoPiece() {
        return Commands.sequence(
            AutoRoutineOnePiece(), 
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedPreload)),
            // Drive back until note is picked
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(AutoConstants.kAuotSpeed, 0, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(2),
            
            // Rotate arm to shooting posiiton while moving forard
            Commands.parallel(
                m_arm.rotateArmCommand(ArmConstants.kArmAngleSubwoofer), // Rotate arm
                m_drive.run(()->m_drive.drive(-AutoConstants.kAuotSpeed, 0, 0, false)) // drive forward
            ).withTimeout(.8),

            Commands.sequence(
                m_drive.runOnce(()->m_drive.drive(0, 0, 0, false)),
                m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5),// Shoot
                m_shooter.runOnce(()->m_shooter.setSpeed(0))
            )
        );
    }

    public Command AutoRoutineThreePiece() {
        return Commands.sequence(
            AutoRoutineTwoPiece(),
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedPreload)),
            // Drive left for third  note
            Commands.parallel(
                m_drive.run(()->m_drive.drive(0, AutoConstants.kAuotSpeed, 0, false)),
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote)
            ).withTimeout(1.9),

            // Drive back until note is picked
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(AutoConstants.kAuotSpeed, 0, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(1),

            // Rotate arm to shooting posiiton while moving forard
            Commands.parallel(
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePreload), // Rotate arm
                m_drive.run(()->m_drive.drive(-AutoConstants.kAuotSpeed,-AutoConstants.kAuotSpeed, 0, false)) // drive forward
            ).withTimeout(1.8),
            m_drive.runOnce(()->m_drive.drive(0, 0, 0, false)),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5),// Shoot
            m_shooter.runOnce(()->m_shooter.setSpeed(0)) 
        );
    }

    public Command AutoRoutineFourPiece() {
        return Commands.sequence(
            AutoRoutineThreePiece(),
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedPreload)),
            // Drive left for third  note
            Commands.parallel(
                m_drive.run(()->m_drive.drive(0, -AutoConstants.kAuotSpeed, 0, false)),
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote)
            ).withTimeout(1.1),

            // Drive back until note is picked
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(AutoConstants.kAuotSpeed, 0, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(1),

            // Rotate arm to shooting posiiton while moving forard
            Commands.parallel(
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePreload), // Rotate arm
                m_drive.run(()->m_drive.drive(-AutoConstants.kAuotSpeed, AutoConstants.kAuotSpeed, 0, false)) // drive forward
            ).withTimeout(.9),
            m_drive.runOnce(()->m_drive.drive(0, 0, 0, false)),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5),// Shoot
            m_shooter.runOnce(()->m_shooter.setSpeed(0)) 
        );
    }

    public Pose2d notePose(double loc[], double xOffset, double yOffset, double angleInDegrees) {
        return new Pose2d(-(loc[1]+yOffset), -(loc[0]+xOffset), Rotation2d.fromDegrees(angleInDegrees));
    }

    public SwerveControllerCommand setSwerveCommand(Trajectory traj){
    
        var thetaController =  new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);  
        SwerveControllerCommand autoTraj = new SwerveControllerCommand(
            traj,
            m_drive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0), 
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_drive::setModuleStates,
            m_drive);
      return autoTraj;
    }

    public SwerveControllerCommand newSwerveControllerCommand(List<Pose2d> poses){

         // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond - 2,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared - 2)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics).setReversed(true);
    
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(poses, config);
        return setSwerveCommand(trajectory);
  }

  public double[] offsetFinalLoc(double[] loc, double offsetX, double offsetY){
    double[] finLoc = {-(loc[1]+offsetY), (loc[0]+offsetX)};
    return finLoc;
  }

  public Command getAutonomousTest() {
    double start[] = {0,0};
    double end[] = {1,1};
    SwerveControllerCommand path1 = 
        newSwerveControllerCommand(List.of(
            notePose(start, 0, 0, 0),
            notePose(end, 0, 0, 0))
        );

    return Commands.sequence(
        new InstantCommand(() -> m_drive.resetOdometry(notePose(start, 0,0, 0))),
        path1,
        new InstantCommand(() -> m_drive.drive(0, 0, 0, false))); 
  }

  public Command getAutonomousSubwoofer213() {
    SwerveControllerCommand SubwooferToNote2 = 
        newSwerveControllerCommand(List.of(
            notePose(Constants.subwooferLocation, 0, 0, 0),
            //notePose(Constants.subwooferLocation, 0, 0, 30),
            notePose(Constants.note2Location, 0, 0,0))
        );
     SwerveControllerCommand Note2Shoot = 
        newSwerveControllerCommand(List.of(
            notePose(Constants.note2Location, 0, 0, 0),
            notePose(Constants.subwooferLocation, 0, 0,0))
        );
    SwerveControllerCommand Note2ToNote1 = 
        newSwerveControllerCommand(List.of(
            notePose(Constants.note2Location, -0.25, 0, 0),
            notePose(Constants.note1Location, 0, 0, 0))
        );
    SwerveControllerCommand Note1Shoot = 
        newSwerveControllerCommand(List.of(
            notePose(Constants.note1Location, 0, 0, 0),
            notePose(Constants.note1Location, -0.25, -0.25,30))
        );
    SwerveControllerCommand Note1ToNote3 = 
        newSwerveControllerCommand(List.of(
            notePose(Constants.note1Location, -.25, -.25, 30),
            notePose(Constants.note3Location, 0, 0, 0))
        );
    SwerveControllerCommand Note3Shoot = 
        newSwerveControllerCommand(List.of(
            notePose(Constants.note1Location, 0, 0, 0),
            notePose(Constants.note1Location, 0.25, 0.25,-30))
        );


    // SwerveControllerCommand autoShoot2 = newSwerveControllerCommand(offsetFinalLoc(Constants.note2Location, 0, -1), 0, 0, 0);
    // SwerveControllerCommand autoPickNote1 = newSwerveControllerCommand(Constants.note1Location, Math.tan(angleForShooting*Math.PI/180), -1, -angleForShooting);
    // SwerveControllerCommand autoShoot1 = newSwerveControllerCommand(offsetFinalLoc(Constants.note1Location, Math.tan(angleForShooting*Math.PI/180), -1), 0, 0, -angleForShooting);    
    // SwerveControllerCommand autoPickNote3 = newSwerveControllerCommand(Constants.note3Location, -Math.tan(angleForShooting*Math.PI/180), -1, angleForShooting);
    // SwerveControllerCommand autoShoot3 = newSwerveControllerCommand(offsetFinalLoc(Constants.note3Location, -Math.tan(angleForShooting*Math.PI/180), -1), 0, 0, angleForShooting);    


    return Commands.sequence(
        new InstantCommand(() -> m_drive.resetOdometry(notePose(Constants.subwooferLocation, 0,0, 0))),
        SubwooferToNote2,
        Note2Shoot,
        //Note2ToNote1,
        //Note1Shoot,
        //Note1ToNote3,
        //Note3Shoot,
        //add paralell command around all the autoPickNote_s and include pick note to both of them
        // new ParallelCommandGroup(
        // add Run Shooter command,
        // autoShoot2, 
        // m_arm.trackLimelightCommand()
        // ),
        // autoPickNote1,
      // new ParallelCommandGroup(
        // add Run Shooter command,
        // autoShoot1, 
        // m_arm.trackLimelightCommand()
        // ),
        // autoPickNote3,
      // new ParallelCommandGroup(
        // add Run Shooter command,
        // autoShoot3, 
        // m_arm.trackLimelightCommand()
        // ),
        new InstantCommand(() -> m_drive.drive(0, 0, 0, false)));
  }


//     public Command getAutonomousSubwooferFar() {
  
//     SwerveControllerCommand autoPickNote8 = newSwerveControllerCommand(Constants.note8Location, -6, 0, 0);
//     double xOffset7[] = {-5.0, -2.0, -1.5, 0.0};
//     double yOffset7[] = {-1.5, -1.5, 0.0,0.0};
//     double angleInDegrees7[] = {0.0,0.0,0.0,0.0};
//     SwerveControllerCommand autoPickNote7 = newSwerveControllerCommand(Constants.note7Location, 
//                         xOffset7, yOffset7, angleInDegrees7);


//     return Commands.sequence(
//         new InstantCommand(() -> m_drive.resetOdometry(new Pose2d(m_robotY,m_robotX,new Rotation2d(0)))),
//         autoPickNote8,
//         autoPickNote7,
//         // autoPickNote3,
//         new InstantCommand(() -> m_drive.drive(0, 0, 0, false)));
  
//   }

}
