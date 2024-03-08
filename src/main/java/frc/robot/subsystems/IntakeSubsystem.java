// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class IntakeSubsystem extends SubsystemBase {
  private CANSparkMax m_intakeMotor;
  private CANSparkMax m_intakeMotor2;

  /** Creates a new Intake. */
  public IntakeSubsystem() {
    m_intakeMotor = new CANSparkMax(IntakeConstants.kLeftIntakeMotorPort, MotorType.kBrushless);
    m_intakeMotor2 = new CANSparkMax(IntakeConstants.kRightIntakeMotorPort, MotorType.kBrushless);
    m_intakeMotor2.follow(m_intakeMotor);
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
  public void hasNote(){ // Should be BooleanSupplier return type
    /*if (){ // return statement later if sensor finds note == true
      return true;
    }
    else {
      return false;
    }*/
    
  }

  // pickNoteCommand
  public Command pickNoteCommand() {
    return runIntakeCommand(0.5); // add .until (hasNote is true) to end
  }
}
 