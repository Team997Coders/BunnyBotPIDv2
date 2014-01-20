/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.templates.RobotMap;


/**
 *
 * @author 997robotics3
 */
public class Dumper extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
     Solenoid extend = new Solenoid(RobotMap.extenddumpersolenoid);
     Solenoid retract = new Solenoid(RobotMap.retreactdumpersolenoid);
 

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public void extenddumper(){
    retract.set(false);
    extend.set(true);
}

public void retractdumper(){
    extend.set(false);
    retract.set(true);
}


public void dumperoff(){
    extend.set(false);
    retract.set(false);
}

}
