/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.commands.BellSpeedThroughTarget;

/**
 * Add your docs here.
 */

public class Vision extends Subsystem {
  NetworkTable table;

  NetworkTableEntry tv;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  boolean isTrackingTarget;
  double xOffset;
  double yOffset;
  double area;

  double mountingHeight;
  double mountingAngle;
  double targetHeight;

  public Vision()
  {
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tv = table.getEntry("tv");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");

    mountingHeight = 12; //inches
    mountingAngle = 0;
    targetHeight = 36;
  }
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new BellSpeedThroughTarget());
  }

  @Override
  public void periodic()
  {
    //read values periodically
    isTrackingTarget = tv.getBoolean(false);
    xOffset = tx.getDouble(0.0);
    yOffset = ty.getDouble(0.0);
    area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putBoolean("LimelightIsTrackingTarget", isTrackingTarget);
    SmartDashboard.putNumber("LimelightXOffset", xOffset);
    SmartDashboard.putNumber("LimelightYOffset", yOffset);
    SmartDashboard.putNumber("LimelightArea", area);
  }

  public double getDistanceToTarget()
  {
    return (targetHeight-mountingHeight)/Math.tan(mountingAngle + yOffset);
  }

  public boolean IsTrackingTarget()
  {
    return isTrackingTarget;
  }
}
