package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.coralState;

public class Intake extends SubsystemBase{
    private TalonFX intakeleft = new TalonFX(12);
    private TalonFX intakepiv = new TalonFX(11);
    private PIDController pid = new PIDController(0.01, 0, 0.);
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    TalonFXConfiguration cfg = new TalonFXConfiguration();
      private Slot0Configs slot0 = cfg.Slot0;
  MotionMagicConfigs motionMagicConfigs = cfg.MotionMagic;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);




    public Intake(){

        slot0.kG = 0.3; // A gear ratio of 4:1 results in 0.25 output
    slot0.kS = 0.3;
    slot0.kV = 0.5; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.02; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 12; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 200; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        400; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 800; // Target jerk of 1600 rps/s/s (0.1 seconds)\

    intakepiv.getConfigurator().apply(cfg);
        intakepiv.setNeutralMode(NeutralModeValue.Brake);



        


    }
    @Override
    public void periodic(){
        if(intakevelocity() > 10){
            Constants.setcoralState(coralState.intake);
            
        }
        else if(intakevelocity() < -10){
            Constants.setcoralState(coralState.scoring);
        }
        else if(intakevelocity() < 48 && intakevelocity() > -5){
            Constants.setcoralState(coralState.intakedpeice);
        }
      
        SmartDashboard.putNumber("intake position",intakepiv.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake Curent", intakeleft.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake Velcoity",intakeleft.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Intake Peice", Constants.getcoralState() == coralState.intakedpeice);
    }
    public double intakeposition(){
        return intakeleft.getPosition().getValueAsDouble();
    }
    public double intakevelocity(){
        return intakeleft.getVelocity().getValueAsDouble();
    }
    public boolean intakevelocitycheck(){//peice is there check
        return (intakevelocity()<48 &&  intakepiv.getPosition().getValueAsDouble()>12);
    }
    
   public Command intakepivoitcmd(double postion){
    return new Command(){
        @Override
        public void initialize(){
            
        }
        @Override
        public void execute(){
            intakepiv.setControl(m_request.withPosition(postion).withEnableFOC(true).withFeedForward(0.6));
        }
        @Override
        public boolean isFinished(){
            return intakevelocitycheck();
        }
        @Override
        public void end(boolean interrupted){
            intakeleft.set(0);
        }
    };
   }


   public Command intakepivoitshootcmd(double postion){
    return new Command(){
        @Override
        public void initialize(){
            
        }
        @Override
        public void execute(){
            intakepiv.setControl(m_request.withPosition(postion).withEnableFOC(true).withFeedForward(0.6));
        }

        @Override
        public boolean isFinished(){
            return intakepiv.getPosition().getValueAsDouble() > postion-0.3 && intakepiv.getPosition().getValueAsDouble() < postion+0.3 ;
        }
        @Override
        public void end(boolean interrupted){
            intakepiv.setControl(m_request.withPosition(postion).withEnableFOC(true).withFeedForward(0.6));
        }

    };
   }
   public Command intakepivoitshootcmdPID(double postion){
    return new Command(){
        @Override
        public void initialize(){
            pid.reset();
            
        }
        @Override
        public void execute(){
            intakepiv.set(-pid.calculate(postion));
        }

        @Override
        public boolean isFinished(){
            return intakepiv.getPosition().getValueAsDouble() > postion-0.1 && intakepiv.getPosition().getValueAsDouble() < postion+0.1 ;
        }
        @Override
        public void end(boolean interrupted){
            intakepiv.setControl(m_request.withPosition(postion).withEnableFOC(true).withFeedForward(0.6));
        }

    };
   }
   public Command intakepivoitshootcmdPID1(double postion){
    return new Command(){
        @Override
        public void initialize(){
            pid.reset();
            
        }
        @Override
        public void execute(){
            intakepiv.set(-pid.calculate(postion));
        }

        @Override
        public boolean isFinished(){
            return false ;
        }
        @Override
        public void end(boolean interrupted){
        }

    };
   }
   public Command intakecmdout(double speed){
    return new Command(){
        @Override
        public void initialize(){
            
        }
        @Override
        public void execute(){
            intakeleft.set(speed);
        }
        @Override
        public boolean isFinished(){
            return false;
        }
    };}
        public Command intakecmdin(double speed){
            return new Command(){
                @Override
                public void initialize(){
                    
                }
                @Override
                public void execute(){
                    intakeleft.set(speed);
                }
                @Override
                public boolean isFinished(){
                    return intakevelocitycheck();
                }
                @Override
                public void end(boolean interrupted){
                    intakeleft.set(0);
                }


    };
   }






    
}
