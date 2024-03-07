// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class Shoot extends Command {
    
  ShooterSubsystem m_shooter;
  double speed;

  public Shoot(ShooterSubsystem m_shooter, double speed) {
    
       this.m_shooter = m_shooter;

        addRequirements(m_shooter);
    }


    @Override
    public void initialize() {
      
    }
    
    @Override
    public void execute() {

    m_shooter.shoot(speed);
   
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
