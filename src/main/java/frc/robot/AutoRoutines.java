package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmConstants;
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
            m_arm.resetCommand().andThen(m_arm.rotateArmCommand(armAngle)) // Rotate arm
        );

    }
    public Command AutoRoutineOnePiece() {
        return Commands.sequence(
            this.autoInit(ShooterConstants.kShooterSpeedSubwoofer, ArmConstants.kArmAngleSubwoofer),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5) // Shoot
        );
    }

    public Command AutoRoutineTwoPiece() {
        return Commands.sequence(
            AutoRoutineOnePiece(), 
           
            // Drive back until note is picked
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(-0.2, 0, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(2),

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


}
