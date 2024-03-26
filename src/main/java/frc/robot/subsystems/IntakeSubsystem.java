// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;



public class IntakeSubsystem extends SubsystemBase {
  private CANSparkFlex m_intakeMotor;
  private Rev2mDistanceSensor distOnboard; 
  private final AnalogInput ultrasonic = new AnalogInput(IntakeConstants.kUltrasonicSensorPort);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final PWMSparkMax m_LEDstrip = new PWMSparkMax(0);

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
   

    SmartDashboard.putData("pickNote", pickNoteCommand(IntakeConstants.kIntakeVoltage));
    SmartDashboard.putData("runIntake", runIntakeCommand(IntakeConstants.kIntakeVoltage));
    SmartDashboard.putData("shootIntake", runIntakeCommand(IntakeConstants.kIntakeVoltage));
  }

  @Override

  public void periodic() {
    distOnboard.setAutomaticMode(true);
    distOnboard.setEnabled(true);
    SmartDashboard.putNumber("SensorDistance", getRange());
    SmartDashboard.putBoolean("hasNote", hasNote());
    distOnboard.setEnabled(true);

    Color detectedColor = m_colorSensor.getColor();
    double IR = m_colorSensor.getIR();
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    int proximity = m_colorSensor.getProximity();

    SmartDashboard.putNumber("Proximity", proximity);

    if (hasNote()) {
      m_LEDstrip.set(0.77);
    } else {
       m_LEDstrip.set(0.61);
    }
  }


  public void setSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
  
  public Command runIntakeCommand(double voltage) {
    return new StartEndCommand(()->this.m_intakeMotor.setVoltage(-voltage), //rmr to chg to speed
                        ()->this.m_intakeMotor.setVoltage(0), this);
  }

  
  public double getRange() {
    return distOnboard.getRange();
    //return ultrasonic.getValue();
  }

  // hasNote
  public boolean hasNote(){ 
    if (m_colorSensor.getProximity() >150 ){
      return true;
    }
    // if (distOnboard.isRangeValid() && distOnboard.getRange() < 7){ 
    //   return true;
    // }
    
    // if (getRange() < 270) {
    //   return true;
    // }
    return false;
  }

  // pickNoteCommand
  public Command pickNoteCommand(double voltage) {
    return runIntakeCommand(voltage).until(this::hasNote);
    }
}
 