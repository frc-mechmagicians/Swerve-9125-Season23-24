package frc.robot.subsystems;


import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class ShooterSubsystem extends SubsystemBase {

    private CANSparkMax m_shooterMotor;
    private CANSparkFlex m_shooterMotor2;

    public ShooterSubsystem(){
        m_shooterMotor = new CANSparkMax(ShooterConstants.kLeftShooterMotorPort, MotorType.kBrushless);
        m_shooterMotor2 = new CANSparkFlex(ShooterConstants.kRightShooterMotorPort, MotorType.kBrushless);
         
        m_shooterMotor.restoreFactoryDefaults();
        m_shooterMotor2.restoreFactoryDefaults();
        
        m_shooterMotor.setIdleMode(IdleMode.kBrake);
        m_shooterMotor2.setIdleMode(IdleMode.kBrake);
 
        m_shooterMotor.setSmartCurrentLimit(30);
        m_shooterMotor2.setSmartCurrentLimit(30);

        SmartDashboard.putNumber("shooterSpeed", 0.7);
        SmartDashboard.putData("runShooter", runShooterCommand(()->
             SmartDashboard.getNumber("shooterSpeed", 0.6)));
    }

    public void setSpeed(double speed) {
        m_shooterMotor.set(-speed*0.55/0.7);
        m_shooterMotor2.set(-
        speed);
    }

    public Command runShooterCommand(DoubleSupplier speed){
        return new StartEndCommand(()->this.setSpeed(speed.getAsDouble()), ()->this.setSpeed(0), this);
    }
    
}
