package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.OperatorConstants;


public class IntakeSubsystem extends SubsystemBase {
  TalonFX m_intakeMotor; 
 VoltageOut m_request = new VoltageOut(0);

  CurrentLimitsConfigs intakeCurrentLimits = new CurrentLimitsConfigs();

  public IntakeSubsystem() {
    m_intakeMotor = new TalonFX(OperatorConstants.IntakeMotorID, "Canivore");

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0; //.25 V outpot to overcome static friction
    slot0Configs.kV = 0; //A velocity target of 1 rps results in a 0.12 V output
    slot0Configs.kA = 0.0; //An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.1; // /An error of 1 prs rsults in 0.11 V output
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

     m_intakeMotor.getConfigurator().apply(slot0Configs);

     intakeCurrentLimits.withStatorCurrentLimitEnable(true);
    intakeCurrentLimits.withStatorCurrentLimit(60); //TODO: Set Current Limit higher if necessary, probably fine at 40Amps
    
  m_intakeMotor.getConfigurator().apply(intakeCurrentLimits);

  }
  @Override
  public void periodic() {
  }

  public void feedIn() {


    //Set Voltage Output
    m_intakeMotor.setVoltage(3);  //TODO: Set Voltage for what works well with the robot
  
  }
  public void feedOut() {

    m_intakeMotor.setVoltage(-1.5); //TODO: Set Voltage for what works well with the robot
  }

  public void ampOut() {
    
    
    m_intakeMotor.setVoltage(-1);   //TODO Tune this value so that it works well with the robot.


  }

  public void stopIntake(){

    m_intakeMotor.setVoltage(0);
  }

public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }
}
