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
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;




public class IntakeSubsystem extends SubsystemBase {
  private CANSparkFlex m_intakeMotor;
  private Rev2mDistanceSensor distOnboard; 

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkFlex(IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    m_intakeMotor.setSmartCurrentLimit(Constants.intakeMotorSmartLimit);
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    m_intakeMotor.restoreFactoryDefaults();
    m_intakeMotor.setSmartCurrentLimit(40);
    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    distOnboard.setAutomaticMode(true);
    distOnboard.setEnabled(true);
    distOnboard.setRangeProfile(RangeProfile.kHighSpeed);
    

    SmartDashboard.putData("pickNote", pickNoteCommand(2));
    SmartDashboard.putData("runIntake", runIntakeCommand(2));
    SmartDashboard.putData("shootIntake", runIntakeCommand(2));
  }

  @Override

  public void periodic() {
    distOnboard.setAutomaticMode(true);
    distOnboard.setEnabled(true);
    SmartDashboard.putNumber("2mDistSensorDistance", distOnboard.getRange());
    SmartDashboard.putBoolean("SensorIsRangeVal", distOnboard.isRangeValid());
    SmartDashboard.putBoolean("SensorIsEnabled", distOnboard.isEnabled());
    distOnboard.setEnabled(true);
  }


  public Command runIntakeCommand(double voltage) {
    return new StartEndCommand(()->this.m_intakeMotor.setVoltage(voltage), //rmr to chg to speed
                        ()->this.m_intakeMotor.setVoltage(0), this);
  }

  public double getRange() {
    return distOnboard.getRange();
  }

  // hasNote
  public boolean hasNote(){ 
    if (distOnboard.isRangeValid() && distOnboard.getRange() < 10){ 
      return true;
    }
    return false;
  }

  // pickNoteCommand
  public Command pickNoteCommand(double voltage) {
    return runIntakeCommand(voltage).until(this::hasNote); // add .until (hasNote is true) to end
  }
}
 