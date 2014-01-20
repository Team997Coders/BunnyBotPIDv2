/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author 997robotics3
 */
public class DriveTrain extends PIDSubsystem {

    private static final double Kp = 0.002;
    private static final double Ki = 0.0;
    private static final double Kd = 0.001;

    // Initialize your subsystem here
    public DriveTrain() {
        super("DriveTrain", Kp, Ki, Kd);
        setAbsoluteTolerance(10);
        getPIDController().setContinuous(false);
        LiveWindow.addActuator("DriveTrain", "PIDSubsystem Controller", getPIDController());
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return 0.0;
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    }
    
     private void SetLeftspeed(double speed){
       SmartDashboard.putNumber("Left: ", speed);
      leftMotor.set(speed);
   } 
   
   private void SetRightspeed(double speed){
       SmartDashboard.putNumber("Right: ", speed);
      rightMotor.set(-speed);
   }

     public void tankDrive(double left, double right){
         SetLeftspeed(left);
         SetRightspeed(right);
     }
    
    
}
