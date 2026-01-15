// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LauncherIOImp;
import frc.robot.subsystems.LauncherSubsystem;
// import frc.robot.subsystems.TurretSubsystem;

public class RobotContainer {
  LauncherSubsystem launcherSubsystem = new LauncherSubsystem(new LauncherIOImp());

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // TurretSubsystem turretSubsystem = new TurretSubsystem(null);
    // Object targetProvider = null;
    // IndexSubsystem indexSubsystem = new IndexSubsystem(null);

    // CommandXboxController controller = new CommandXboxController(1);
    // controller.rightBumper().whileTrue(
    // launcherSubsystem.stayOnTarget(targetProvider)
    // .alongWith(turretSubsystem.stayOnTarget(targetProvider))
    // .alongWith(
    // indexSubsystem.feedToLauncher(() -> launcherSubsystem.onTarget() &&
    // turretSubsystem.onTarget())));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
