// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class RobotContainer {

  /* Drive Controls */
  // private DefaultTeleopSub s_DefaultTeleopSub = DefaultTeleopSub.getInstance();
  private Swerve s_Swerve = Swerve.getInstance();
  // private LEDSubsystem s_LEDSubsystem = LEDSubsystem.getInstance();
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kStart.value);
  
  public RobotContainer() {
    configureBindings();
  }

  public Swerve getSwerve() {
    return s_Swerve;
  }

  // public LEDSubsystem getLEDSub() {
  //   return s_LEDSubsystem;
  // }

  private void configureBindings() {
    s_Swerve.setDefaultCommand(new DefaultTeleop(driver, operator));
  }

}