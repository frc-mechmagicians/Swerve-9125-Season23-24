package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.Encoder;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    private Encoder m_armEncoder;
    private double m_armOffset = 0;

    // use some constant in Constants.java
    PIDController m_armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg,ArmConstants.kv,ArmConstants.ka);

    public ArmSubsystem() {

        m_pivotMotor = new CANSparkMax(ArmConstants.kLeftPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2 = new CANSparkMax(ArmConstants.kRightPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor2.restoreFactoryDefaults();
        //m_pivotMotor.setInverted(true);

        m_pivotMotor.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.setIdleMode(IdleMode.kBrake);
        m_pivotMotor.setSmartCurrentLimit(30); //Make a max current limit
        m_pivotMotor2.setSmartCurrentLimit(30); //Make a max current limit
        
        
        m_armEncoder = new Encoder(ArmConstants.kEncoderPort1,ArmConstants.kEncoderPort2);
        m_armEncoder.setDistancePerPulse(ArmConstants.kArmEncoderDistancePerPulse);
        m_armPID.setTolerance(0.5);

        // Add to smart dashboard
        SmartDashboard.putNumber("ArmAngle", 0);
        SmartDashboard.putData("rotateArm", this.rotateArmCommand(SmartDashboard.getNumber("ArmAngle", 10)));
        SmartDashboard.putData("trackLimelight", this.trackLimelightCommand());
        SmartDashboard.putData("ResetArm", resetCommand());
        

    }

    @Override

    public void periodic(){
        SmartDashboard.putNumber("ArmPosition", armPosition());
        SmartDashboard.putNumber("setpoint", m_armPID.getSetpoint());
    }

    public void setSpeed(double speed) {
        if (speed > 6) {
            speed = 6;
        }
        if (speed< -5){
            speed = -5;
        }
        SmartDashboard.putNumber("setSpeed", speed);
        m_pivotMotor.setVoltage(-speed);
        m_pivotMotor2.setVoltage(speed);
    }


    public double armPosition() {
        return m_armEncoder.getDistance();
    }

    public Command resetCommand() {
        return Commands.sequence(
            run(()->setSpeed(-5)).withTimeout(.75),
            run(()->setSpeed(-5)).until(this::isStopped).withTimeout(1),
            run(()->setSpeed(0)).withTimeout(.2),
            runOnce(()->m_armEncoder.reset()),
            runOnce(()->this.setPos(0)),
            runOnce(()->setSpeed(0))
        );
    }

 
    public boolean isStopped(){
        //return m_armEncoder.getStopped();
        if(Math.abs(m_armEncoder.getRate())*ArmConstants.kArmEncoderDistancePerPulse < 1){
          return true;
        }
        return false;
    }

    public void setPos(double angle) {
        m_armPID.setSetpoint(angle);
    }

    public Command rotateArmCommand(double angle) {
        return run(()->{   
            m_armPID.setSetpoint(angle); 
            this.setSpeed(m_armPID.calculate(this.armPosition()) +
                m_feedforward.calculate(Math.PI*this.armPosition()/180, 
                Math.PI/180*m_armEncoder.getRate()*ArmConstants.kArmEncoderDistancePerPulse));
         }).until(m_armPID::atSetpoint);
    }

    public Command trackLimelightCommand() {
        return run(()-> {
            SmartDashboard.putNumber("voltage", m_feedforward.calculate(Math.PI*this.armPosition()/180, 
                Math.PI/180*m_armEncoder.getRate()*ArmConstants.kArmEncoderDistancePerPulse));        
            
            if (Limelight.isAprilTagDetected()) {
                m_armPID.setSetpoint(Limelight.readLimelightAngle());
            }
            //setSpeed(m_armPID.calculate(this.armPosition()));
            setSpeed(m_armPID.calculate(this.armPosition()) +
             m_feedforward.calculate(Math.PI*this.armPosition()/180, 
                Math.PI/180*m_armEncoder.getRate()*ArmConstants.kArmEncoderDistancePerPulse));
        }).until(m_armPID::atSetpoint);  
    }

    public void resetArmEncoder(){
        m_armEncoder.reset();
    }
}
