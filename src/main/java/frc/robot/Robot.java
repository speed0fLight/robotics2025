// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.*;

import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.TimedRobot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  // "CANSparkMax" was changed to SparkMax. Hi Steve!
  private final SparkMax m_rightMotor = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax m_rightMotor_follower = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax m_leftMotor = new SparkMax(4, MotorType.kBrushed);
  private final SparkMax m_leftMotor_follower = new SparkMax(5, MotorType.kBrushed);
  
  SparkMaxConfig cimConfig = new SparkMaxConfig();
  

// Configure primary encoder for brushed motor
//cimConfig.encoder.countsPerRevolution(8192).inverted(true);
  
//cim.configure(cimConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  //private final CANSparkJNI m_leftMotor = new CANSparkMax(2, MotorType.kBrushed);
private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
private final Joystick m_stick = new Joystick(0);

  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
   }

   @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
    m_rightMotor_follower.setInverted(true);
    m_leftMotor.setInverted(true);
    m_leftMotor.setInverted(true);
    m_rightMotor.setIdleMode(IdleMode.kBrake); // this stuff is currently broken, working on porting it to the latest package.
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor_follower.follow(m_leftMotor);
    m_rightMotor_follower.follow(m_rightMotor);

    // some docs below for sparkmax --vv
    //https://docs.revrobotics.com/revlib/spark/configuring-a-spark

    // Lift_02.setIdleMode(IdleMode.kCoast);
    // Lift_03.setIdleMode(IdleMode.kCoast);
    // Lift_04.setIdleMode(IdleMode.kCoast);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-m_stick.getY(), -m_stick.getX());
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
