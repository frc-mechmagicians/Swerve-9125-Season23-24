package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    

    public RelativeEncoder armEncoder;
    PIDController armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);

    
     // use some constant in Constants.java


    public ArmSubsystem() {
        m_pivotMotor = new CANSparkMax(ArmConstants.kLeftPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2 = new CANSparkMax(ArmConstants.kRightPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.follow(m_pivotMotor);
        armEncoder = m_pivotMotor.getEncoder();
        m_pivotMotor.setSmartCurrentLimit(Constants.generalMotorSmartLimit); //Make a max current limit
        m_pivotMotor2.setSmartCurrentLimit(Constants.generalMotorSmartLimit); //Make a max current limit
        armPID.setTolerance(1);
    }

    @Override
    public void periodic(){

    }

    public Command positionArmCommand(double position){
        final double position2 = position * Math.PI/180; // rmr to create a function that gets the current position with limelight later
        return new StartEndCommand(()-> this.m_pivotMotor.set(armPID.calculate(armEncoder.getPosition(), position2) 
        + feedforward.calculate(position2, ArmConstants.feedforwardVelocity)), ()-> this.m_pivotMotor.set(0), this);
    }

    public Command rotateArmCommand(double speed){
        return new StartEndCommand(()->this.m_pivotMotor.set(speed), ()->this.m_pivotMotor.set(0), this);
    }

    
}
