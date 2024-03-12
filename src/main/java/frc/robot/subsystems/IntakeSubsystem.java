// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;




public class IntakeSubsystem extends SubsystemBase {
  private CANSparkFlex m_intakeMotor;
  private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distMXP;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    m_intakeMotor.setSmartCurrentLimit(Constants.intakeMotorSmartLimit);
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distMXP = new Rev2mDistanceSensor(Port.kMXP);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setSmartCurrentLimit(40);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    distOnboard.setAutomaticMode(true);
    distMXP.setAutomaticMode(true);

    SmartDashboard.putData("pickNote", pickNoteCommand());
      SmartDashboard.putData("runIntake", runIntakeCommand(0.3));
    SmartDashboard.putNumber("2mDistSensorDistance", distOnboard.getRange());
    SmartDashboard.putBoolean("SensorIsRangeVal", distOnboard.isRangeValid());
    SmartDashboard.putBoolean("kOnboardEnabled?", distOnboard.isEnabled());
    SmartDashboard.putNumber("12mDistSensorDistance", distMXP.getRange());
    SmartDashboard.putBoolean("1SensorIsRangeVal", distMXP.isRangeValid());
    SmartDashboard.putBoolean("1kOnboardEnabled?", distMXP.isEnabled());
  }

  @Override

  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeCommand(double speed) {
    return new StartEndCommand(()->this.m_intakeMotor.set(0.3), //rmr to chg to speed
                        ()->this.m_intakeMotor.set(0), this);
  }

  // hasNote
  public boolean hasNote(){ 
    if (distOnboard.isRangeValid() && distOnboard.getRange() < 2){ 
      return true;
    }
    return false;
  }

  // pickNoteCommand
  public Command pickNoteCommand() {
    return runIntakeCommand(0.3).until(this::hasNote); // add .until (hasNote is true) to end
  }
}
 