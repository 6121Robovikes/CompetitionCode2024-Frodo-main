// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OperatorConstants;

public class PivotSubsystem extends SubsystemBase {

  TalonFX m_pivotMotor;
  FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
  CurrentLimitsConfigs pivotCurrentLimits = new CurrentLimitsConfigs();

  private final PositionVoltage m_request = new PositionVoltage(0, 0, false, 0, 0, false, false, false);


  public PivotSubsystem() {

    m_pivotMotor = new TalonFX(OperatorConstants.PivotMotorID, "Canivore"); 
    
    
    // Factory default on motors
    m_pivotMotor.getConfigurator().apply(new TalonFXConfiguration());
   

     // Motion Profile Position
    var slot0Configs = new Slot0Configs();
    //Per https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/basic-pid-control.html#position-control
    //Recommend no S,V,A values yet.
    slot0Configs.kG = 0.01; //value of gravity
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kP = 1.5; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    m_pivotMotor.getConfigurator().apply(slot0Configs);

    pivotFeedbackConfigs.withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);
    
    m_pivotMotor.getConfigurator().apply(pivotFeedbackConfigs);

    pivotCurrentLimits.withStatorCurrentLimit(40); //TODO: Set Current Limit higher if necessary
    pivotCurrentLimits.withStatorCurrentLimitEnable(true);

    m_pivotMotor.getConfigurator().apply(pivotCurrentLimits);


  }
  public void setPosition(double position) {
    
  TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

// send the request to the device
    m_request.Position = m_setpoint.position;
    m_request.Velocity = m_setpoint.velocity;
    m_pivotMotor.setControl(m_request.withPosition(position).withSlot(0));


   }

   public void pivotDiagnostics(){
    
    System.out.println("Current Pivot Error From Setpoint: " + m_pivotMotor.getClosedLoopError());
   }
   
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
