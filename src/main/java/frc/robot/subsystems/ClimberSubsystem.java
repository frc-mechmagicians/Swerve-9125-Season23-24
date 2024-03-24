package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase{
  // private CANSparkMax mLeftClimberMotor;
  // private CANSparkMax mRightClimberMotor;
  
  // private SparkPIDController mLeftClimberPID;
  // private SparkPIDController mRightClimberPID;

  // private RelativeEncoder mLeftClimberEncoder;
  // private RelativeEncoder mRightClimberEncoder;

  // public ClimberSubsystem(){
  //   mLeftClimberMotor = new CANSparkMax(Constants.ClimberConstants.kClimberLeftMotorId, MotorType.kBrushless);
  //   mRightClimberMotor = new CANSparkMax(Constants.ClimberConstants.kClimberRightMotorId, MotorType.kBrushless);

  //   mLeftClimberPID = mLeftClimberMotor.getPIDController();
  //   mLeftClimberPID.setP(Constants.ClimberConstants.kClimberP);
  //   mLeftClimberPID.setI(Constants.ClimberConstants.kClimberI);
  //   mLeftClimberPID.setD(Constants.ClimberConstants.kClimberD);
  //   mLeftClimberPID.setOutputRange(Constants.ClimberConstants.kClimberMinOutput, Constants.ClimberConstants.kClimberMaxOutput);

  //   mRightClimberPID = mRightClimberMotor.getPIDController();
  //   mRightClimberPID.setP(Constants.ClimberConstants.kClimberP);
  //   mRightClimberPID.setI(Constants.ClimberConstants.kClimberI);
  //   mRightClimberPID.setD(Constants.ClimberConstants.kClimberD);
  //   mRightClimberPID.setOutputRange(Constants.ClimberConstants.kClimberMinOutput, Constants.ClimberConstants.kClimberMaxOutput);

  //   mLeftClimberEncoder = mLeftClimberMotor.getEncoder();
  //   mLeftClimberEncoder.setPositionConversionFactor(Constants.ClimberConstants.kClimberGearRatio);
  //   mLeftClimberEncoder.setVelocityConversionFactor(Constants.ClimberConstants.kClimberGearRatio);

  //   mRightClimberEncoder = mRightClimberMotor.getEncoder();
  //   mRightClimberEncoder.setPositionConversionFactor(Constants.ClimberConstants.kClimberGearRatio);
  //   mRightClimberEncoder.setVelocityConversionFactor(Constants.ClimberConstants.kClimberGearRatio);

  //   mLeftClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  //   mRightClimberMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

  //   mLeftClimberMotor.setInverted(false);
  //   mRightClimberMotor.setInverted(true);
  // }
  // public void setSpeed(double speedLeft, double speedRight){
  //  mLeftClimberMotor.set(speedLeft); 
  //  mRightClimberMotor.set(speedRight); 
  // }
  // public Command climbersUp(){
  //   return new StartEndCommand(()-> this.setSpeed(0.4, 0.4), ()->this.setSpeed(0,0), this);
  // }

  //   public Command climbersDown(){
  //   return new StartEndCommand(()-> this.setSpeed(-0.4, -0.4), ()->this.setSpeed(0,0), this);
  // }
//   public void Climb(){
//    mLeftClimberMotor.set(0.4); 
//    mRightClimberMotor.set(0.4); 
//   }
//   public void climbDown(){
//     mLeftClimberMotor.set(-0.4);
//     mRightClimberMotor.set(-0.4);
//   }
//   public void StopClimber(){
//     mLeftClimberMotor.set(0);
//     mRightClimberMotor.set(0);
//   }
}
