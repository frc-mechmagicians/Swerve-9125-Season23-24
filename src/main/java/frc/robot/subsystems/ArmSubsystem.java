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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.Encoder;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    private Encoder m_armEncoder;
    private double m_pos;

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
        m_armPID.setTolerance(1);
        
        m_pos = 0;

        // Add to smart dashboard
        SmartDashboard.putNumber("ArmAngle", 0);
        SmartDashboard.putData("rotateArm", this.rotateArmCommand(SmartDashboard.getNumber("ArmAngle", 10)));
        SmartDashboard.putData("trackLimelight", this.trackLimelightCommand());
        SmartDashboard.putData("ResetArm", this.runOnce(()->this.reset()));
    }

    @Override

    public void periodic(){
        SmartDashboard.putNumber("ArmPosition", armPosition());
    }

    public void setSpeed(double speed) {
        if (speed > 2) {
            speed = 2;
        }
        if (speed< -1){
            speed = -1;
        }
        SmartDashboard.putNumber("setSpeed", speed);
        m_pivotMotor.setVoltage(-speed);
        m_pivotMotor2.setVoltage(speed);
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

    /*
    public Command positionArmCommand(DoubleSupplier angleInDegrees){
        m_pos = angleInDegrees.getAsDouble();
        return new FunctionalCommand(()->{}, 
            ()->this.setSpeed(m_armPID.calculate(armPosition(), angleInDegrees.getAsDouble())),
            reason -> this.setSpeed(0), 
            ()-> false, this);
    }*/


    public Command rotateArmCommand(double angle){
        return new InstantCommand(()->m_pos = angle);
       // m_pos = angle;
        // return trackLimelightCommand();
    }

    public double getPos() {
        return m_pos;
    }

    public void setPos(double angle) {
        m_pos = angle;
    }

    public void trackLimelight() {
        //double angle = Limelight.readLimelightAngle()
        double output = m_armPID.calculate(armPosition(), m_pos);
        setSpeed(output);
     }
    public Command trackLimelightCommand(){
        return new FunctionalCommand(()->{}, ()->this.trackLimelight(), 
            x->this.setSpeed(0), ()->{return false;}, this);
    }
}
