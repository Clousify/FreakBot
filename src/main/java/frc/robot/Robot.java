// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private final DifferentialDrive m_robotDrive;
  private final CommandXboxController driver1Controller = new CommandXboxController(0);

  private final SparkMax m_leftMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax m_leftMotor2 = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax m_rightMotor = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax m_rightMotor2 = new SparkMax(1, MotorType.kBrushless);

  private SparkMaxConfig rightMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig rightMotor2Config = new SparkMaxConfig();
  private SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
  private SparkMaxConfig leftMotor2Config = new SparkMaxConfig();

  /** Called once at the beginning of the robot program. */
  public Robot() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    rightMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    
    rightMotor2Config
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .follow(m_rightMotor);

    leftMotorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(true);
    
    leftMotor2Config
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .follow(m_leftMotor);

    m_robotDrive = new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.arcadeDrive(-driver1Controller.getLeftY(), -driver1Controller.getRightX());
  }
}
