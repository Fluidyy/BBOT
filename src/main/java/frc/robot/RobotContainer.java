// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
          .withDeadband(MaxSpeed * 0.3).withRotationalDeadband(MaxAngularRate * 0.3) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();



  private final CommandXboxController driverJoy = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


  private Intake intake = new Intake();

  private Climb climb = new Climb();
  public RobotContainer() {
    


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

    operator.leftTrigger(0.2).whileTrue(new ParallelCommandGroup(intake.intakecmd(0.5), intake.intakepivoitcmd(13.999))).whileFalse(new ParallelCommandGroup(intake.intakecmd(0.02), intake.intakepivoitcmd(.5)));
    operator.rightTrigger(0.2).whileTrue(intake.intakepivoitshootcmd(3.9404296875).andThen(intake.intakecmd(-0.75))).whileFalse(new ParallelCommandGroup(intake.intakecmd(0), intake.intakepivoitcmd(0.5)));
    operator
        .y()
        .whileTrue(new SequentialCommandGroup(intake.intakepivoitcmd(3.9404296875),climb.cmdspeed(1)))//change val
        .whileFalse(climb.cmdspeed(0)); 

        operator
        .a()
        .whileTrue(new ParallelCommandGroup(intake.intakepivoitcmd(3.9404296875),climb.cmdspeed(-1)))
        .whileFalse(climb.cmdspeed(0));


    

  }
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
