package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooterMotor;
    private CANSparkMax m_shooterMotor2;

    public ShooterSubsystem(){
        m_shooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkMax(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
        m_shooterMotor2.setInverted(true);
        m_shooterMotor2.follow(m_shooterMotor);
    }

    public Command runShooterCommand(double speed){
        return new StartEndCommand(()->this.m_shooterMotor.set(speed), ()->this.m_shooterMotor.set(0), this);
    }
    
}
