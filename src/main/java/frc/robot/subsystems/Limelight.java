// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Test code to change between pipelines on limelight
public class Limelight extends InstantCommand {
  private NetworkTable limelight;
  private int pipelineIndex = 0;
  private Joystick joystick;
  private JoystickButton ybutton;

  public Limelight(){
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    joystick = new Joystick(0);
    ybutton = new JoystickButton(joystick, 4);
    // ybutton.onTrue(switchPipeline());
    // ybutton.whenPressed(new InstantCommand(switchPipeline()))
    // CommandXboxController controllercommand = new CommandXboxController(1);
    // Trigger xButton = controllercommand.x();

  }

  public void switchPipeline(){
    pipelineIndex = (pipelineIndex + 1) % 2;
    limelight.getEntry("pipeline").setNumber(pipelineIndex);
  }

  public int getPipeLineIndex(){
    return pipelineIndex;
  }
}