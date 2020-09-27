/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import frc.robot.commands.FlagWave;
import frc.robot.commands.FlagStop;
import frc.robot.commands.StraightDrive;
import frc.robot.commands.FaceAngle;
//import frc.robot.commands.TrackReflectiveTape;;

public class OI 
{
  public OI()
  {
    // Put button mapped commands here.
    // Example: Robot.m_IO.mechButton1.whenPressed(new exampleCommand1());
    
    Robot.m_io.aButton.whenPressed(new FlagWave());
    Robot.m_io.bButton.whenPressed(new FlagStop());
    Robot.m_io.yButton.whileHeld(new StraightDrive());
    if (Robot.m_io.getGamepadDpadAngle() != -1){
      switch (Robot.m_io.getGamepadDpadAngle()){
        case 0: new FaceAngle(0);
        break;
        case 90: new FaceAngle(90);
        break;
        case 180: new FaceAngle(180);
        break;
        case 270: new FaceAngle(270);
        break;
      }
    }
    //Robot.m_io.xButton.whenPressed(new TrackReflectiveTape());
  }
}
