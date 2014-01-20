package edu.wpi.first.wpilibj.templates;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
    
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.CounterBase.EncodingType; import edu.wpi.first.wpilibj.PIDSource.PIDSourceParameter;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import java.util.Vector;
/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
   public static final int compressorSpike = 1; 
   public static final int compressorpresureswitch = 1;
   public static int extenddumpersolenoid ; 
   public static final int retreactdumpersolenoid = 4;
    public final static double GyroKp = 0.03;
   
    public static double gyroFactor = .1;
    public static int leftVictorSlot = 9;
    public static int rightVictorSlot = 10;
    public static int leftEncoderSlot1 = 5;
    public static int leftEncoderSlot2 = 6;
    public static int rightEncoderSlot1 = 3;
    public static int rightEncoderSlot2 = 4;
    public static int gyroSlot = 1;
    public static double gyroSensitivity = 0.007;
    public static int dumperSolenoidExtend = 2;
    public static int dumperSolenoidRetract = 4;
    public static int kickerSolenoidExtend = 1;
    public static int kickerSolenoidRetract = 3;
    public static int ultrasonicSlot = 4;
   
    
    
   
}

