/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
/* Procedure
1) Given: Currently tracking a target
2) Find where the target's position on the x-axis is
3) Drive/rotate to the center
*/

public class AlignToTarget extends Command {
  public AlignToTarget() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.m_vision);
    requires(Robot.m_drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.m_vision.IsTrackingTarget()) {
        if (Robot.m_vision.TargetPos()[0] < -3) {
            Robot.m_drivetrain.driveByPercent(.2, -.2);
        }      
        else if (Robot.m_vision.TargetPos()[0] > 3) {
            Robot.m_drivetrain.driveByPercent(-.2, .2);         
        }
        else {
          end();
        }
    }
    else {
      System.out.println("Cannot find target.");
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.m_drivetrain.driveByPercent(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
