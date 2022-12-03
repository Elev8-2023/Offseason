package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  public final TalonFX shooterFlywheel = new TalonFX(Constants.shooterPort);
  public static final TalonFX shooterHood = new TalonFX(Constants.hoodPort);
  public final TalonFX loadBall = new TalonFX(Constants.loadBallPort);

  public double shooterRPM, t_rpm, hood_ogpos;


  public ShooterSubsystem() {

    /*Shooter Flywheel*/{
      shooterFlywheel.configFactoryDefault();
      shooterFlywheel.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      shooterFlywheel.setSensorPhase(true);
      shooterFlywheel.setInverted(false);
      shooterFlywheel.configNominalOutputForward(0, 30);
      shooterFlywheel.configNominalOutputReverse(0, 30);
      shooterFlywheel.configPeakOutputForward(1, 30);
      shooterFlywheel.configPeakOutputReverse(-1, 30);
      shooterFlywheel.configAllowableClosedloopError(0, 0, 30);
      shooterFlywheel.config_kP(0, 0.38, 30);
      shooterFlywheel.config_kD(0, 10.0, 30);
      shooterFlywheel.config_kI(0, 0.0, 30);
      shooterFlywheel.config_kF(0, 0.0673, 30);
      shooterFlywheel.configClosedloopRamp(2);
    }
      
    /*Shooter Hood*/ {
      shooterHood.configFactoryDefault();
      shooterHood.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
      shooterHood.setSensorPhase(true);
      shooterHood.setInverted(false);
      shooterHood.configNominalOutputForward(0, 30);
      shooterHood.configNominalOutputReverse(0, 30);
      shooterHood.configPeakOutputForward(1, 30);
      shooterHood.configPeakOutputReverse(-1, 30);
      shooterHood.configAllowableClosedloopError(0, 150.0, 30);
      shooterHood.config_kP(0, 0.15, 30);
      shooterHood.config_kD(0, 0.0005, 30);
      shooterHood.config_kI(0, 0.001, 30);
      shooterHood.config_kF(0, 0.0, 30);
      shooterHood.configClosedloopRamp(0);
    }
    
  }

  @Override
  public void periodic() {
    shooterRPM = (600*shooterFlywheel.getSelectedSensorVelocity())/2048;
    //t_rpm = 600/2048*turret.getSelectedSensorVelocity();
    hood_ogpos = shooterHood.getSelectedSensorPosition()/Constants.hoodReduction;
    SmartDashboard.putNumber("shooter rpm", shooterRPM); //may want to add gear reductions
    SmartDashboard.putNumber("turret rpm", t_rpm); //may want to add gear reductions
    SmartDashboard.putNumber("Hood Position Degrees", hood_ogpos);
    SmartDashboard.putNumber("Hood Position Sensor Positions", shooterHood.getSelectedSensorPosition());
    SmartDashboard.putNumber("Hood Setpoint Sensor Positions", 20*Constants.hoodReduction);
    // This method will be called once per scheduler run
  }

  public void shootRPM(double rpm) {
    shooterFlywheel.set(TalonFXControlMode.Velocity, (rpm*2048)/600);
    
    if (shooterRPM >= .95*rpm)
    {
      loadBall.set(TalonFXControlMode.PercentOutput, 0.5);
    }
  }

  public void turretPow(double pow)
  {
    //turret.set(TalonFXControlMode.PercentOutput, pow);
  }

  public void hoodAngle(double angle)
  {
    //pass
    if (angle>=40)
    {angle = 40;}
    if (angle<=0)
    {angle = 0;}

    shooterHood.set(TalonFXControlMode.Position, angle*Constants.hoodReduction);
  }

}
