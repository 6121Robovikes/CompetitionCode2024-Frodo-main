package frc.robot.subsystems;


import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OperatorConstants;

public class ClimberSubsystem extends SubsystemBase{

    TalonFX m_leftClimb; 
    TalonFX m_rightClimb;

    public ClimberSubsystem() {
        m_leftClimb = new TalonFX(OperatorConstants.ClimberLeftMotorID, "Canivore"); 
        m_rightClimb = new TalonFX(OperatorConstants.ClimberRigtMotorID, "Canivore");
    
        m_leftClimb.setInverted(false);
        m_rightClimb.setInverted(false);


        //factory default both motors
        m_leftClimb.getConfigurator().apply(new TalonFXConfiguration());
        m_rightClimb.getConfigurator().apply(new TalonFXConfiguration());

        //position control
        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0; // An error of 0.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.; // A velocity of 1 rps results in 0.1 V output

        m_leftClimb.getConfigurator().apply(slot0Configs);
        m_rightClimb.getConfigurator().apply(slot0Configs);

  }

    @Override
  public void periodic() {

  }

  public void climbExtend(double position) {

    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    //feedforward is realy hard math so don't do it yet
    m_leftClimb.setControl(m_request.withPosition(position).withFeedForward(0));
    m_rightClimb.setControl(m_request.withPosition(position).withFeedForward(0));

 }
 
 public double getPosition() {

    System.out.println("left climb Postion"+ m_leftClimb.getRotorPosition());
    return m_leftClimb.getRotorPosition().getValueAsDouble();
 }

  public void stop(){
  
  }

  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

}
