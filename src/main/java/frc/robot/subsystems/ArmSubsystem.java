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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmSubsystem extends SubsystemBase{
    private CANSparkMax m_pivotMotor;
    private CANSparkMax m_pivotMotor2;
    private DutyCycleEncoder m_armEncoder;

    PIDController m_armPID = new PIDController(ArmConstants.kP, ArmConstants.kI, ArmConstants.kD);
    ArmFeedforward m_feedforward = new ArmFeedforward(ArmConstants.ks, ArmConstants.kg, ArmConstants.kv, ArmConstants.ka);

     // use some constant in Constants.java


    public ArmSubsystem() {

        m_pivotMotor = new CANSparkMax(ArmConstants.kLeftPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor2 = new CANSparkMax(ArmConstants.kRightPivotMotorPort, MotorType.kBrushless);
        m_pivotMotor.restoreFactoryDefaults();
        m_pivotMotor2.restoreFactoryDefaults();
        m_pivotMotor2.setInverted(true);

        m_pivotMotor.setIdleMode(IdleMode.kBrake);
        m_pivotMotor2.setIdleMode(IdleMode.kBrake);
        m_pivotMotor.setSmartCurrentLimit(30); //Make a max current limit
        m_pivotMotor2.setSmartCurrentLimit(30); //Make a max current limit
        
        m_armEncoder = new  DutyCycleEncoder(new DigitalInput(ArmConstants.kEncoderPort));
        m_armPID.setTolerance(1);

        // Add to smart dashboard
        DoubleSupplier angleSupplier = ()->SmartDashboard.getNumber("armAngleInDegrees", 0);
        SmartDashboard.putData("positionArm", this.positionArmCommand(angleSupplier.getAsDouble()));
        SmartDashboard.putData("trackLimelight", this.trackLimelightCommand());

    }

    @Override

    public void periodic(){
        SmartDashboard.putNumber("ArmPosition", armPosition());
        SmartDashboard.putNumber("ArmPosition_dist)", m_armEncoder.getDistance());
        SmartDashboard.putNumber("ArmPosition_get)", m_armEncoder.get());
        SmartDashboard.putNumber("ArmPosition_posoff)", m_armEncoder.getPositionOffset());
        SmartDashboard.putNumber("ArmPosition_rot)", m_armEncoder.getDistancePerRotation());
    }

    public void setSpeed(double speed) {
        m_pivotMotor.set(speed);
        m_pivotMotor2.set(speed);
    }

    public double armPosition() {
        return m_armEncoder.getDistance();
    }

    public Command positionArmCommand(double angleInDegrees){
        double position = angleInDegrees * Math.PI/180;
        return new PIDCommand(m_armPID, this::armPosition, position, output -> this.setSpeed(output), this);
    }

    public Command rotateArmCommand(double speed){
        return new StartEndCommand(()->this.setSpeed(speed), ()->this.setSpeed(0), this);
    }

    public void trackLimelight() {
        double angle = Limelight.readLimelightAngle();
        double output = m_armPID.calculate(armPosition(),angle);
        setSpeed(output);
     }
    public Command trackLimelightCommand(){
        return new FunctionalCommand(()->{}, ()->this.trackLimelight(), 
            x->this.setSpeed(0), ()->{return false;}, this);
    }
}
