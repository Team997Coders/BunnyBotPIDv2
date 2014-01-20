/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.templates.RobotMap;


/**
 *
 * @author 997robotics3
 */
public class SubCompressor extends Subsystem {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
 Compressor mycompressor1 = new Compressor(RobotMap.compressorpresureswitch,RobotMap.compressorSpike);
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
      public void CompressorOn(){
        
        mycompressor1.start(); 
            
    }
    
    public void CompressorOff(){
        mycompressor1.stop();
    }
    
}
