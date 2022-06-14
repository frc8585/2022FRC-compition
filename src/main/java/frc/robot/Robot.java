// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//import java.sql.Time;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private final Joystick m_stick = new Joystick(0);

  private final WPI_TalonSRX M1 = new WPI_TalonSRX(1);
  private final WPI_TalonSRX M2 = new WPI_TalonSRX(2);
  private final WPI_TalonSRX M3 = new WPI_TalonSRX(3);
  private final WPI_TalonSRX M4 = new WPI_TalonSRX(4);
  
  private final CANSparkMax M5 = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax M6 = new CANSparkMax(21, MotorType.kBrushed);
  private final WPI_VictorSPX M7 = new WPI_VictorSPX(2);

  private final MotorControllerGroup leftMotor = new MotorControllerGroup(M3, M4);
  private final MotorControllerGroup rightMotor = new MotorControllerGroup(M1, M2);
  private final DifferentialDrive drive = new DifferentialDrive(leftMotor,rightMotor);
  private final Encoder M6_encoder = new Encoder(0, 1);

  public int state1 =0;
  public int state2 =0;
  public int state3 =0;
  public int state4 =0;
  public int state5=0;
  public double rsh;
  public double rdh=rsh-465;
  private double startTime;


  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotor.setInverted(true);
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    M5.set(0);
    M6.set(0);
    M7.set(0);
     startTime= Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("starttime", startTime);

    M5.set(1);
    double time =Timer.getFPGATimestamp();
    if(time-startTime>3 && time-startTime<= 8)
    drive.tankDrive(-0.5, -0.5);
    else{
      drive.stopMotor();
    }     
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {
    M5.set(0);
    M6.set(0);
    M7.set(0);
    leftMotor.set(0);
    rightMotor.set(0);
    rsh =M6_encoder.get();

  }

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
     SmartDashboard.putBoolean("a", m_stick.getRawButton(1));
     SmartDashboard.putBoolean("b", m_stick.getRawButton(2));
     SmartDashboard.putBoolean("x", m_stick.getRawButton(3));
     SmartDashboard.putBoolean("y", m_stick.getRawButton(4));
     SmartDashboard.putBoolean("lft", m_stick.getRawButton(5));
     SmartDashboard.putBoolean("rit", m_stick.getRawButton(6));
     SmartDashboard.putNumber("M6encoder", M6_encoder.get());
     SmartDashboard.putNumber("M5", M5.get());
     SmartDashboard.putNumber("M6", M6.get());
     SmartDashboard.putNumber("M7", M7.get());
     SmartDashboard.putNumber("forward", m_stick.getY());
     SmartDashboard.putNumber("rught&&left", m_stick.getX());
     SmartDashboard.putNumber("intakepg", m_stick.getRawAxis(5));

     M6.set(m_stick.getRawAxis(5));
        
  
     M5.set(m_stick.getRawAxis(3)+m_stick.getRawAxis(2)*-1);

     drive.arcadeDrive(m_stick.getY()*-1, m_stick.getX());
 
     if(m_stick.getRawButton(3)){
      M7.set(-1);}
     else if(m_stick.getRawButton(4)){
      M7.set(1);}
     else{
        M7.set(0);
     }
}
    
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
