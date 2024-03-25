// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.equation.Variable;

import edu.wpi.first.units.Voltage;

//import java.lang.invoke.ClassSpecializer.SpeciesData;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class Ground extends Command {

    PivotSubsystem m_pivot;
    IntakeSubsystem m_intake;
   
    double groundPosition = 93; //determine this position 
    double stowPosition = 1;
    Integer voltage;

    


  public Ground(PivotSubsystem m_pivot, IntakeSubsystem m_intake)   
  {
    
    this.m_pivot = m_pivot;
    this.m_intake = m_intake;
  

    addRequirements(m_pivot, m_intake); //
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  m_pivot.setPosition(groundPosition);
  m_intake.feedIn();

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_pivot.setPosition(stowPosition);

  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    m_intake.getVoltage();
    //if (voltage = 0) {
    //  m_pivot.setPosition(stowPosition);
    //  return true;
    //}
    return false;
  } 

}