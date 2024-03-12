package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.Encoder;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    private Encoder m_armEncoder;


     // use some constant in Constants.java
     PIDController m_armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg,ArmConstants.kv,ArmConstants.ka);

    public ArmSubsystem() {

        m_pivotMotor = new CANSparkMax(ArmConstants.kLeftPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2 = new CANSparkMax(ArmConstants.kRightPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor2.restoreFactoryDefaults();
        m_pivotMotor.setInverted(true);

        m_pivotMotor.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.setIdleMode(IdleMode.kBrake);
        m_pivotMotor.setSmartCurrentLimit(30); //Make a max current limit
        m_pivotMotor2.setSmartCurrentLimit(30); //Make a max current limit
        
        
        m_armEncoder = new Encoder(ArmConstants.kEncoderPort1,ArmConstants.kEncoderPort2);
        m_armEncoder.setDistancePerPulse(ArmConstants.kArmEncoderDistancePerPulse);
        m_armPID.setTolerance(1);

        // Add to smart dashboard
        SmartDashboard.putNumber("ArmAngle", 0);
        SmartDashboard.putData("positionArm", this.positionArmCommand(()->SmartDashboard.getNumber("ArmAngle", 10)));
        SmartDashboard.putData("trackLimelight", this.trackLimelightCommand());
        SmartDashboard.putData("ResetArm", this.runOnce(()->this.reset()));
    }

    @Override

    public void periodic(){
        SmartDashboard.putNumber("ArmPosition", armPosition());
    }

    public void setSpeed(double speed) {
        if (speed > 0.2) {
            speed = 0.2;
        }
        if (speed< -0.05){
            speed = -0.05;
        }
        SmartDashboard.putNumber("setSpeed", speed);
        m_pivotMotor.set(speed);
        m_pivotMotor2.set(speed);
    }


    public double armPosition() {
        return m_armEncoder.getDistance();
    }
    public void reset(){
        m_armEncoder.reset();
    }
    public boolean isStopped(){
        return m_armEncoder.getStopped();
    }
    public Command positionArmCommand(DoubleSupplier angleInDegrees){
    
        return new FunctionalCommand(()->{}, 
            ()->this.setSpeed(m_armPID.calculate(armPosition(), angleInDegrees.getAsDouble())),
            reason -> this.setSpeed(0), 
            ()-> false, this);
    }


    public Command rotateArmCommand(double speed){
        return new StartEndCommand(()->this.setSpeed(speed), ()->this.setSpeed(0), this);
    }
   
    public void trackLimelight() {
        //double angle = Limelight.readLimelightAngle();
        double angle = 35;

        double output = m_armPID.calculate(armPosition(),angle);
        setSpeed(output);
     }
    public Command trackLimelightCommand(){
        return new FunctionalCommand(()->{}, ()->this.trackLimelight(), 
            x->this.setSpeed(0), ()->{return false;}, this);
    }
}
