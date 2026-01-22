// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeIOImp;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LauncherIOImp;
import frc.robot.subsystems.LauncherSubsystem;

public class RobotContainer {
  Mechanism2d robotMechanism = new Mechanism2d(Units.inchesToMeters(30), Units.inchesToMeters(30));
  LauncherSubsystem launcherSubsystem = new LauncherSubsystem(new LauncherIOImp());
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new IntakeIOImp());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    CommandXboxController controller = new CommandXboxController(0);
    controller.a().onTrue(intakeSubsystem.intake());
    controller.b().onTrue(intakeSubsystem.outtake(2.0));
    controller.x().onTrue(intakeSubsystem.stow());
    controller.y().onTrue(intakeSubsystem.stop());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
