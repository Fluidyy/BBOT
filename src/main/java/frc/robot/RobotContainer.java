// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.units.measure.LinearVelocity;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.RobotCentric RobotCentric_Translation = new SwerveRequest.RobotCentric();



  private final CommandXboxController driverJoy = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
      private final SendableChooser<Command> autoChooser;



  private Intake intake = new Intake();

  private Climb climb = new Climb();
  public RobotContainer() {

    NamedCommands.registerCommand("Shoot", intake.intakepivoitshootcmd(3.9404296875).andThen(intake.intakecmdout(-0.75)).withTimeout(2));
    NamedCommands.registerCommand("Intake", new ParallelCommandGroup(intake.intakecmdin(0.5), intake.intakepivoitcmd(13.999)));
    autoChooser = AutoBuilder.buildAutoChooser("Leave");
        SmartDashboard.putData("Auto Mode", autoChooser);
    


    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() ->
          drive.withVelocityX(driverJoy.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
              .withVelocityY(driverJoy.getLeftX() * MaxSpeed) // Drive left with negative X (left)
              .withRotationalRate(-driverJoy.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
              )
  );
  driverJoy.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
//intake
 driverJoy.a().onTrue(drivetrain.applyRequest(()-> brake));


    driverJoy.pov(0).whileTrue(drivetrain.applyRequest(() -> RobotCentric_Translation.withVelocityX(1).withVelocityY(0)));
    driverJoy.pov(90).whileTrue(drivetrain.applyRequest(() -> RobotCentric_Translation.withVelocityX(0).withVelocityY(1)));
    driverJoy.pov(180).whileTrue(drivetrain.applyRequest(() -> RobotCentric_Translation.withVelocityX(-1).withVelocityY(0)));
    driverJoy.pov(270).whileTrue(drivetrain.applyRequest(() -> RobotCentric_Translation.withVelocityX(0).withVelocityY(-1)));

    operator.leftTrigger(0.2).whileTrue(new ParallelCommandGroup(intake.intakecmdin(0.5), intake.intakepivoitcmd(13.999))).whileFalse(new ParallelCommandGroup(intake.intakecmdin(0.02), intake.intakepivoitcmd(.5)));
    operator.rightTrigger(0.2).whileTrue(intake.intakepivoitshootcmd(3.9404296875).andThen(new ParallelCommandGroup(intake.intakecmdout(-0.75),intake.intakepivoitshootcmd(3.9404296875)))).whileFalse(new ParallelCommandGroup(intake.intakecmdin(0), intake.intakepivoitcmd(0.5)));
    operator
        .y()
        .whileTrue(new SequentialCommandGroup(intake.intakepivoitshootcmd(3.9404296875),climb.cmdspeed(1,0)))//change val
        .whileFalse(climb.cmdspeed(0,0)); 

        operator
        .a()
        .whileTrue(new ParallelCommandGroup(intake.intakepivoitcmd(3.9404296875),climb.cmdspeed(-1,0)))
        .whileFalse(climb.cmdspeed(0,0));

        operator
        .b()
        .whileTrue(new SequentialCommandGroup(intake.intakepivoitshootcmd(3.9404296875),climb.cmdspeed(0,-1)))//change val
        .whileFalse(climb.cmdspeed(0,0)); 
     

  }
  

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}