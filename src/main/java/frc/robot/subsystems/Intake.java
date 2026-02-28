package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Units.*;

// add shuffle board adjustible values for intake 

public class Intake extends SubsystemBase{
    public final TalonFX m_intake_roller_motor = new TalonFX(53);
    public final TalonFX m_intake_arm_motor = new TalonFX(52);

    // private final AngularVelocity intake_rpm;

      //PID constants (subject to change - can change via smartdashboard)
  private double kP = SmartDashboard.getNumber("intake kP", 0.01);
  private double kD = SmartDashboard.getNumber("intake kD", 0);
  private double kI = SmartDashboard.getNumber("intake kI", 0);
  private double kFF = SmartDashboard.getNumber(" intake kFF", 12.0 / 5767.0); // feedforward constant based on max voltage and max RPM of the motors
  private double kMaxOutput = SmartDashboard.getNumber("intake kMaxOutput", 1.0);
  private double kV = SmartDashboard.getNumber("shooter kV", 0);
  private double kMinOutput = SmartDashboard.getNumber("intake kMinOutput", -1.0);
  private double speed = 0;
   boolean updatePID = false;
      
   double newkP = SmartDashboard.getNumber("intake kP", kP);
   double newkI = SmartDashboard.getNumber("intake kI", kI);
   double newkD = SmartDashboard.getNumber("intake kD", kD);
   double newkFF = SmartDashboard.getNumber("intake kFF", kFF);

  public final Angle out_position = Rotations.of(40);
  private final VoltageOut m_VoltageOut = new VoltageOut(0);


// velocity and potition cotrol for ctre motor::done i think
    public TalonFXConfiguration intakeConfig() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = 0.11;
        config.Slot0.kV = 0.12;
        config.Slot0.kA = 0.01;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.Slot0.kS = 0.25;

        config.MotionMagic.MotionMagicAcceleration = 0; 
        config.MotionMagic.MotionMagicJerk = 0;

        return config;
    }
    public void motorconfig() {
        TalonFXConfiguration config = intakeConfig();
        m_intake_roller_motor.getConfigurator().apply(config);
        m_intake_arm_motor.getConfigurator().apply(config);

    }
    //private final ShuffleboardTab tab = Shuffleboard.getTab("intake_1");
    //private final ShuffleboardTab tab2 = Shuffleboard.getTab("intake_2");
    //private GenericEntry intakespeed_1 = tab.add("intake 1 speed", speed).withPosition(0, 0).withSize(2, 1).getEntry();
    //private GenericEntry intakespeed_2 = tab2.add("intake 2 speed", speed).withPosition(0, 0).withSize(2, 1).getEntry();
     boolean intakespeed = SmartDashboard.putNumber("intake speed 1 ", speed);
     boolean intakespeed02 = SmartDashboard.putNumber("intake speed 2 ", speed);
    
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    public void intakestart(){
        m_intake_roller_motor.setControl(velocityRequest.withVelocity(speed));
        m_intake_arm_motor.setControl(velocityRequest.withVelocity(speed));
    }

    public  void rollerstop(){
        m_intake_roller_motor.setControl(velocityRequest.withVelocity(0));
        m_intake_arm_motor.setControl(velocityRequest.withVelocity(0));
    
    }
    private final PositionTorqueCurrentFOC m_positiontorque = new PositionTorqueCurrentFOC(out_position);
   //work on position
   public void intake_out() {
    m_intake_roller_motor.setControl(m_positiontorque.withPosition(0));
    m_intake_arm_motor.setControl(m_positiontorque.withPosition(0));
   }
   public void intake_in() {
     m_intake_roller_motor.setControl(m_positiontorque.withPosition(0));
     m_intake_arm_motor.setControl(m_positiontorque.withPosition(0));
   }
    
   

    public SequentialCommandGroup intaketoggle() {
        return new RunCommand(this::intakestart, this).andThen(this::rollerstop);
    }
}