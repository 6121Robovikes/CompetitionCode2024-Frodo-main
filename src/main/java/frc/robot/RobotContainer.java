// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.FeedAmp;
import frc.robot.commands.ClimbExtend;
import frc.robot.commands.ClimbRetract;
import frc.robot.commands.ClimbStop;
import frc.robot.commands.CommandSwerveDrivetrain;
import frc.robot.commands.Shoot;
import frc.robot.commands.Source;
import frc.robot.commands.Stow;
import frc.robot.commands.Ground;
import frc.robot.commands.LoadTheShooter;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.commands.FeedAmp;

public class RobotContainer {

   //robot's  subsystems are defined here
 // public static Joystick m_joystick = new Joystick(1);
  private final CommandJoystick m_controller =new CommandJoystick(1);
  private static final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private static final PivotSubsystem m_pivot = new PivotSubsystem();
  private static final IntakeSubsystem m_intake = new IntakeSubsystem();
  private static final ClimberSubsystem m_climb = new ClimberSubsystem();
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SendableChooser<Command> autoChooser;
 
  //private Command runAuto = drivetrain.getAutoPath("Stay Still and Shoot");

  //drivetrain stuff... don't mess with it
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController m_driverController = new CommandXboxController(0); // My controller

  public RobotContainer() {
    
  // Register Named Commands
  NamedCommands.registerCommand("Load", new LoadTheShooter(m_pivot, m_intake).withTimeout(3));
  NamedCommands.registerCommand("Ground", new Ground(m_pivot, m_intake).withTimeout(4));
  NamedCommands.registerCommand("Stow", new Stow(m_pivot, m_intake).withTimeout(1));

  
    
    configureBindings();
   autoChooser =AutoBuilder.buildAutoChooser();
   SmartDashboard.putData("autoChooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();


   
  }



  // My drivetrain

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(0.1).withRotationalDeadband(0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  


  private final Telemetry logger = new Telemetry(MaxSpeed);



  private void configureBindings() {

    //drivetrain bindings
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

   
      m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
      m_driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

  
    //OPERATOR BINDINGS

    m_controller.button(7).whileTrue(new Shoot(m_shooter, 0));
    m_controller.button(5).onTrue(new Source(m_pivot, m_intake));
    m_controller.button(2).onTrue(new Ground(m_pivot, m_intake));
    m_controller.button(1).onTrue(new LoadTheShooter(m_pivot, m_intake).withTimeout(5));
    m_controller.button(8).onTrue(new FeedAmp(m_intake, m_pivot).withTimeout(3));
    m_controller.button(11).whileTrue(new ClimbExtend(m_climb));
    m_controller.button(11).onFalse(new ClimbStop(m_climb));
    m_controller.button(12).whileTrue(new ClimbRetract(m_climb));
    m_controller.button(12).onFalse(new ClimbStop(m_climb));
    m_controller.button(3).whileTrue(new Stow(m_pivot, m_intake));
    
    //Default Commands to run when no other commands require the subsystem. Used to stop motors when not needed anymore
    m_intake.setDefaultCommand(new Stow(m_pivot, m_intake));
    m_climb.setDefaultCommand(new ClimbStop(m_climb));
    

    //default shooterspeed
    m_shooter.setDefaultCommand(new Shoot(m_shooter, 65)); //adjust to 65 for competition


  }

  public void autoDrive(double xSpeed, double ySpeed, double rotation){
    drivetrain.applyRequest(() -> drive.withVelocityX(xSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(ySpeed) // Drive left with negative X (left)
            .withRotationalRate(rotation)); // Drive counterclockwise with negative X (left)

  }
}

  //public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    //return runAuto;


