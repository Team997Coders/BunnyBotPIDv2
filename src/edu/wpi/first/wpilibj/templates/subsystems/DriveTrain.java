/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.RobotMap;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.templates.commands.TankDrive;

/**
 *
 * @author 997robotics3
 */
public class DriveTrain extends PIDSubsystem {
    
    Gyro driveGyro; 
    SpeedController leftMotor;
    SpeedController rightMotor;
    Encoder leftEncoder; 
    Encoder rightEncoder; 
    AnalogChannel myUltrasonic;
    private static final double Kp = 0.002;
    private static final double Ki = 0.0;
    private static final double Kd = 0.001;

    // Initialize your subsystem here
    public DriveTrain() {
        super("DriveTrain", Kp, Ki, Kd);
        driveGyro = new Gyro(RobotMap.gyroSlot);
        leftMotor = new Victor(RobotMap.leftVictorSlot);
        rightMotor = new Victor(RobotMap.rightVictorSlot);
        leftEncoder = new Encoder(RobotMap.leftEncoderSlot1, RobotMap.leftEncoderSlot2);
        rightEncoder = new Encoder(RobotMap.rightEncoderSlot1, RobotMap.rightEncoderSlot2);
        myUltrasonic = new AnalogChannel(RobotMap.ultrasonicSlot);
        
        setAbsoluteTolerance(10);
        getPIDController().setContinuous(false);
        LiveWindow.addActuator("DriveTrain", "PIDSubsystem Controller", getPIDController());
        leftEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);   
        leftEncoder.start();
        rightEncoder.setPIDSourceParameter(PIDSource.PIDSourceParameter.kDistance);
        rightEncoder.start();
        myUltrasonic.setAverageBits(8);
        
        
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
        distance = myUltrasonic.getVoltage()/.009766;
        SmartDashboard.putNumber("raw distance ", myUltrasonic.getVoltage());
        SmartDashboard.putNumber("Analog: ", distance);
        return myUltrasonic.getVoltage();
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
