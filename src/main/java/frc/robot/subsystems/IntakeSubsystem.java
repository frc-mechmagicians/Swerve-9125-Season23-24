// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;




public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor;
  private Rev2mDistanceSensor distOnboard; 
  private Rev2mDistanceSensor distMXP;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kLeftIntakeMotorPort, MotorType.kBrushless);
    m_intakeMotor.setSmartCurrentLimit(Constants.intakeMotorSmartLimit);
    distOnboard = new Rev2mDistanceSensor(Port.kOnboard);
    distMXP = new Rev2mDistanceSensor(Port.kMXP);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runIntakeCommand(double speed) {
    return new StartEndCommand(()->this.m_intakeMotor.set(speed), 
                        ()->this.m_intakeMotor.set(0), this);
  }

  // hasNote
  public boolean hasNote(){ 
    if (distMXP.getRange() < 2 && distOnboard.getRange() < 2){ 
      return true;
    }
    return false;
  }

  // pickNoteCommand
  public Command pickNoteCommand() {
    return runIntakeCommand(0.5).until(()->this.hasNote()==true);
  }
}
 