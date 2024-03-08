package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    

    public RelativeEncoder armEncoder;
    PIDController armPID = new PIDController(0, 0, 0);

    
     // use some constant in Constants.java


    public ArmSubsystem() {
        m_pivotMotor = new CANSparkMax(ArmConstants.kLeftPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2 = new CANSparkMax(ArmConstants.kRightPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2.follow(m_pivotMotor);
        armEncoder = m_pivotMotor.getEncoder();
        m_pivotMotor.setSmartCurrentLimit(0); //Make a max current limit
        armPID.setTolerance(1);
    }

    @Override
    public void periodic(){

    }

    public Command positionArmCommand(double position){
        return new StartEndCommand(()-> this.m_pivotMotor.set(armPID.calculate(armEncoder.getPosition(), position)), ()-> this.m_pivotMotor.set(0), this);
    }

    public Command rotateArmCommand(double speed){
        return new StartEndCommand(()->this.m_pivotMotor.set(speed), ()->this.m_pivotMotor.set(0), this);
    }

    
}
