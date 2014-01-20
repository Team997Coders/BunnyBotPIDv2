/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc997.BunnyBot2013.RobotMap;

/**
 *
 * @author 997robotics3
 */
public class DriveTrain extends PIDSubsystem {
    
    Gyro driveGyro = RobotMap.driveGyro; 
    SpeedController leftMotor = RobotMap.driveTrainLeftMotor;
    SpeedController rightMotor = RobotMap.driveTrainRightMotor;
    RobotDrive robotDrive21 = RobotMap.driveTrainRobotDrive21;
    Encoder leftEncoder = RobotMap.driveTrainLeftEncoder;
    Encoder rightEncoder = RobotMap.driveTrainRightEncoder;
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
        setDefaultCommand(new TankDrive());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return leftEncoder.pidGet();
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);   
     SmartDashboard.putNumber("SetMotor: ", output);
        leftMotor.pidWrite(output * (.25-GyroAdjustmentSaturation()));
        rightMotor.pidWrite(-output* (.25+GyroAdjustmentSaturation()));
    
    }
    
    public double fmod(double a, double n){
        return (double) (a%n);
           
    }
    
     public void UpdateDashBoard(){
        double angle;
        angle = fmod(driveGyro.getAngle(), 360);
        SmartDashboard.putNumber("Gyro: ", angle);
        SmartDashboard.putNumber("leftEncoder: ", leftEncoder.getRaw());
        SmartDashboard.putNumber("rightEncoder: ", rightEncoder.getRaw());
        this.ReadSensor();
    }
  
    public double ReadSensor(){
        double distance;
        distance = RobotMap.myUltrasonic.getVoltage()/.009766;
        SmartDashboard.putNumber("raw distance ", RobotMap.myUltrasonic.getVoltage());
        SmartDashboard.putNumber("Analog: ", distance);
        return RobotMap.myUltrasonic.getVoltage();
    }
    
    public void ResetSensers(){
        leftEncoder.reset();
        rightEncoder.reset();
        driveGyro.reset();
        
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
     
       public void reset(){
       this.disable();
       tankDrive(0, 0);
       ResetSensers();
   }
   
     private double GyroAdjustmentSaturation() {
       if (driveGyro.getAngle()*RobotMap.gyroFactor>.1) {
           return .1;
           
       } else if (driveGyro.getAngle()*RobotMap.gyroFactor<-.1) {
           return -.1;
       }else {
           return driveGyro.getAngle()*RobotMap.gyroFactor;
       }
    }

    
}
