// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team.gif.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimitSwitch extends SubsystemBase {
  private static DigitalInput LimitSwitch;



  /** Creates a new ExampleSubsystem. */
  public LimitSwitch() {
    LimitSwitch = new DigitalInput (9);
  }

 public boolean State1() {
   return LimitSwitch.get();
 }


}
