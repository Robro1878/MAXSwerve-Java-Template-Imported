// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants.DriveCommandConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.utils.LimelightHelpers;

public class AlignToTag extends Command {
  ProfiledPIDController x_Controller;
  ProfiledPIDController y_Controller;
  DriveSubsystem m_drive;
  Pose3d botPose3D;
  Pose2d botPose2D;
  /** Creates a new AlignToTag. */
  public AlignToTag(DriveSubsystem drive) {
    addRequirements(drive);
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_Controller = new ProfiledPIDController(DriveCommandConstants.kXP, DriveCommandConstants.kXI, DriveCommandConstants.kXD, 
    new Constraints(DriveCommandConstants.kMaxVelocity, DriveCommandConstants.kMaxAccel));
    y_Controller = new ProfiledPIDController(DriveCommandConstants.kYP, DriveCommandConstants.kYI, DriveCommandConstants.kYD, 
    new Constraints(DriveCommandConstants.kMaxVelocity, DriveCommandConstants.kMaxAccel));
    x_Controller.setGoal(DriveCommandConstants.xGoal);
    y_Controller.setGoal(DriveCommandConstants.yGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    botPose3D = LimelightHelpers.getBotPose3d_TargetSpace("");
    botPose2D = botPose3D.toPose2d();
    double x_output = x_Controller.calculate(botPose2D.getX());
    double y_output = y_Controller.calculate(botPose2D.getY());

    m_drive.drive(x_output, y_output, 0, false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
