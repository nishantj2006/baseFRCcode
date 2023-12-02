package com.team254.frc2023.subsystems;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  public enum GameObject {
    CUBE(new Color(0.143, 0.427, 0.429)),
    CONE(new Color(0.361, 0.524, 0.113)),
    UNKNOWN(new Color(0, 0, 0));

    private Color objectColor;

    public Color getGameObjectColor() {
      return objectColor;
    }
    private GameObject(Color color) {
      this.objectColor = color;
    }
  }

  public GameObject gameObject = GameObject.UNKNOWN;

  private TalonSRX intakeMotor;

  private DigitalInput bannerIntake;
  private boolean didSeeBanner;

  // public ColorSensorV3 intakeColorSensor;
  private ColorMatch intakeColorMatch;
 
  public IntakeType cIntakeType = IntakeType.LOW;

  // Different intake types
  // High - Human Player Station
  // Low - When you lower the intake right on the game element
  // Ground - You can intake the cone when it is flipped
  public enum IntakeType {
    LOW, HIGH, GROUND;
  }
  
  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new TalonSRX(22);

    
    

  }



  public void runIntake(double powDouble) {
  
    intakeMotor.set(ControlMode.PercentOutput, powDouble);


  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}