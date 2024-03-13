package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public Command AutoRoutineOnePiece() {
        return Commands.sequence(
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedSubwoofer)), // Start shooter moter
            m_arm.rotateArmCommand(ArmConstants.kArmAngleSubwoofer), // Rotate arm
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5) // Shhoot
        );
    }

    public Command AutoRoutineTwoPiece() {
        return Commands.sequence(
            AutoRoutineOnePiece(), 
            m_shooter.runOnce(()->m_shooter.setSpeed(ShooterConstants.kShooterSpeedPreload)), // Increase shooter speed
            Commands.deadline(
                m_intake.pickNoteCommand(IntakeConstants.kIntakeVoltage), // pick note
                m_drive.run(()->m_drive.drive(0, -0.2, 0, false)), // drive back
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePickNote) // Rotate arm
            ).withTimeout(2),
            Commands.deadline(
                m_arm.rotateArmCommand(ArmConstants.kArmAnglePreload), // Rotate arm
                m_drive.run(()->m_drive.drive(0, 0.2, 0, false)) // drive forward
            ),
            m_intake.runIntakeCommand(IntakeConstants.kIntakeVoltage).withTimeout(0.5) // Shoot
        );
    }
}
