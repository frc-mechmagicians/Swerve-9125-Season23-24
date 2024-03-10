package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    private DutyCycleEncoder m_armEncoder;

    PIDController armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    ArmFeedforward feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);

     // use some constant in Constants.java


    public ArmSubsystem() {
        m_pivotMotor = new CANSparkMax(ArmConstants.kLeftPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2 = new CANSparkMax(ArmConstants.kRightPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.follow(m_pivotMotor);
        m_pivotMotor.setSmartCurrentLimit(Constants.generalMotorSmartLimit); //Make a max current limit
        m_pivotMotor2.setSmartCurrentLimit(Constants.generalMotorSmartLimit); //Make a max current limit

        m_armEncoder = new  DutyCycleEncoder(new DigitalInput(ArmConstants.kEncoderPort));
        armPID.setTolerance(1);

        // Add to smart dashboard
        DoubleSupplier angleSupplier = ()->SmartDashboard.getNumber("armAngleInDegrees", 0);
        SmartDashboard.putData("positionArm", this.positionArmCommand(angleSupplier.getAsDouble()));
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("ArmPoistion", armPosition());
    }

    public double armPosition() {
        return m_armEncoder.getAbsolutePosition();
    }

    public Command positionArmCommand(double angleInDegrees){
        double position = angleInDegrees * Math.PI/180;
        return new PIDCommand(armPID, this::armPosition, position, output->m_pivotMotor.set(output), this);
    }

    public Command rotateArmCommand(double speed){
        return new StartEndCommand(()->this.m_pivotMotor.set(speed), ()->this.m_pivotMotor.set(0), this);
    }
}
