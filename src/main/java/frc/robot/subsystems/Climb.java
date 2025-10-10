package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
// import frc.robot.subsystems.lookuptable.setpoint;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class  Climb extends SubsystemBase {

  final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

  private TalonFX climbPivot = new TalonFX(9);
  private TalonFX climbWheels = new TalonFX(10);

  private PIDController pidup = new PIDController(0.05, 0, 0);

  private CANcoder climbe = new CANcoder(50, "Drivetrain");
  // idk
  TalonFXConfiguration cfg = new TalonFXConfiguration();

  private Slot0Configs slot0 = cfg.Slot0;

  MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;

  // idk rn #dont use this id for this motor

  private int distance;
  private Timer time3 = new Timer();


  public Climb() {
    // cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // cfg.Feedback.FeedbackRemoteSensorID = climbe.getDeviceID();
    climbPivot.setNeutralMode(NeutralModeValue.Brake);
    slot0.kG = 0.2; // A gear ratio of 4:1 results in 0.25 output
    slot0.kS = 0.25;
    slot0.kV = 0.1; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 7; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0.1; // no output for integrated error
    slot0.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 100; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        300; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 900; // Target jerk of 1600 rps/s/s (0.1 seconds)\

    climbPivot.getConfigurator().apply(cfg);

    climbPivot.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Error", pidup.getPositionError());

    SmartDashboard.putNumber("Velocity", climbPivot.getVelocity().getValueAsDouble());

    SmartDashboard.putNumber("clibm", climbPivot.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("Encoder Climb", climbe.getAbsolutePosition().getValueAsDouble());
  }

  public boolean check() {
    return distance > 75;
  }

  public double getDistance() {
    return distance;
  }

  public void speed(double speed) {
    climbPivot.set(speed);
  }

  public double velocity() {
    return climbPivot.getVelocity().getValueAsDouble();
  }

  public Command cmd(double position) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        climbPivot.setControl(m_request.withPosition(position).withEnableFOC(true));
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command cmdspeed(double speed) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        climbWheels.set(0.67);

        climbPivot.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        climbWheels.set(0);
      }
      

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command pid(double position) {
    return new Command() {
      @Override
      public void initialize() {

        pidup.reset();
        pidup.setSetpoint(position);
      }

      @Override
      public void execute() {

        double speed = pidup.calculate(climbPivot.getPosition().getValueAsDouble());
        climbPivot.set(speed);
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command down(double position) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {

        if (Math.abs(climbPivot.getPosition().getValueAsDouble()) < 4.058349609375) {

          climbPivot.set(-0.7);
        }
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public Command up(double position) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {

        if (climbPivot.getPosition().getValueAsDouble() < 2.023) {

          climbPivot.set(0.7);
        }
      }

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        return false; // Check if the setpoint is reached
      }
    };
  }

  public boolean hasVelocity(double inputVelo) {

    double Velo = climbPivot.getVelocity().getValueAsDouble();

    double velocitythreshold = inputVelo;

    return Math.abs(Velo) > velocitythreshold;
  }

  public boolean hasVelocityautoalighn() {

    double Velo = climbPivot.getVelocity().getValueAsDouble();

    double velocitythreshold = 20;

    return Math.abs(Velo) > velocitythreshold;
  }

  public Command autoncmdOut(double speed, double Velocity) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {
        climbPivot.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        climbPivot.set(0);
      }

      @Override
      public boolean isFinished() {
        return hasVelocity(Velocity);
      }
    };
  }

  public Command autoncmdIn(double speed) {
    return new Command() {
      @Override
      public void initialize() {
        time3.reset();
        time3.start();
      }

      @Override
      public void execute() {
        climbPivot.set(speed);
      }

      @Override
      public void end(boolean interrupted) {
        climbPivot.set(0);
      }

      @Override
      public boolean isFinished() {
        return time3.get() > 1.25 && Math.abs(climbPivot.getVelocity().getValueAsDouble()) < 24;
      }
    };
  }
}
