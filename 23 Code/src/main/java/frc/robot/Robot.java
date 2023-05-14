/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//test 1
//test 2
//test 3

package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController; //old drive chain
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive; //old drive chain
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup; //old drive chain
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort; 
import edu.wpi.first.wpilibj.*;
//new limelight imports -also uses smart dash
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private MecanumDrive m_robotDrive;
///right motors
 private final VictorSP rightMotor1 = new VictorSP(1);
 private final VictorSP rightMotor2 = new VictorSP(0);
  //left motors
  private final VictorSP leftMotor1 = new VictorSP(3);
  private final VictorSP leftMotor2 = new VictorSP(2);

//may need to invert right motors .setInverted(true);

  //arm Controller Motor
  private final VictorSP elbowMotor = new VictorSP(4);
  private final VictorSP wristMotor = new VictorSP (6); 
  private final VictorSP clapMotor = new VictorSP(5);


  //joystick
  Joystick stick =new Joystick(0);
  XboxController xstick = new XboxController(1);
  private boolean buttonPressed;

//PID+Encoder
Encoder encoder = new Encoder(0,1);
PIDController pid = new PIDController(0.01, 0, 0);
int setpoint = 0;

//pneumatics
private final Compressor comp = new Compressor(null);
private final DoubleSolenoid solenoid = new DoubleSolenoid(null, 0, 1);

//drive encoder
private final Encoder driveEncoder = new Encoder(2,3,true,EncodingType.k4X );
private final double kDriveTick2Feet = 1.0 / 128 *6 *Math.PI / 12;

//limelight
NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
NetworkTableEntry tx = table.getEntry("tx");
NetworkTableEntry ty = table.getEntry("ty");
NetworkTableEntry ta = table.getEntry("ta");
double targetArea = 5.0; //sets distance away from object/turn to 5 pixels -need to tune later
PIDController turningController = new PIDController(0.1, 0, 0);
PIDController movingController = new PIDController(0.1, 0, 0);
 

 @Override
  public void robotInit() {
    
    buttonPressed = false;
    CameraServer.startAutomaticCapture();
    m_robotDrive = new MecanumDrive(leftMotor2, leftMotor2, rightMotor1, rightMotor2);

  }
  

  @Override
  public void robotPeriodic() {
SmartDashboard.putNumber("Encoder Value", encoder.get() * kDriveTick2Feet);

  }


  @Override
  public void autonomousInit() {
  

//set variables
  driveEncoder.reset();
  double setpoint = 0;
}


  @Override
  public void autonomousPeriodic() {


  //joystick command to move 10 feet
if (stick.getRawButton(1)) {
setpoint = 10;
}else if (stick.getRawButton(2)){
setpoint = 0;
double errorSum = 0;
double lastTimeStamp = 0;
lastTimeStamp = Timer.getFPGATimestamp();
  }


//get sensor position
double sensorPosition = driveEncoder.get() *kDriveTick2Feet;

//PID Calculations
double kP = 0.1;
double error = setpoint - sensorPosition;
double outputSpeed = kP *error;
double driveSpeed = outputSpeed;

//Motor Output
m_robotDrive.driveCartesian(0, driveSpeed, 0);

  }


  @Override
  public void teleopInit() {


this.encoder.reset();
this.setpoint = 0;
comp.disable();

  }



  @Override
  public void teleopPeriodic() {

      // solenoid controls left/right bumpers on xbox change firing directions
    if (xstick.getLeftBumperPressed()) {
      solenoid.set(DoubleSolenoid.Value.kForward);
    } else if (xstick.getRightBumperPressed()) {
      solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    // Compressor control with xstick paddles buuton 11 is left and 12 right
    if (xstick.getRawButton(11)) {
      comp.isEnabled();
    } else if (xstick.getRawButton(12)) {
      comp.disable();

/* Can scale drive speed if needed old drive chain code
double driveSpeedScale = -0.9; // Change this value to adjust the drive speed scale

    double forwardSpeed = stick.getY();
    double turnSpeed = stick.getZ();


    // Scale the joystick values to reduce the robot's speed
    forwardSpeed *= driveSpeedScale;
    turnSpeed *= driveSpeedScale; */




    // Drive the robot using the scaled joystick values
    m_robotDrive.driveCartesian(-stick.getY(), -stick.getX(), -stick.getZ());

    Timer.delay(0.005); // Small delay to avoid overloading the driver station

    if (xstick.getRawButton(4)) {
      elbowMotor.set(0.40);
    } else if (xstick.getRawButton(3)) {
      elbowMotor.set(-0.15);
    } else if (xstick.getRawButton(5)){
      elbowMotor.set(0.1);
    } else {
      elbowMotor.set(0.0);
    }



    if (xstick.getRawButton(9)) {
      clapMotor.set(0.25);
    } else if (xstick.getRawButton(10)) {
      clapMotor.set(-0.25);
    } else {
      clapMotor.set(0.0);
    } 


    if (xstick.getRawButton(7)){ //when button 7 is pushed revert back to old code
      if (xstick.getRawButton(2)) {
        wristMotor.set(0.25);
      } else if (xstick.getRawButton(1)) {
        wristMotor.set(-0.15);
      } else {
        wristMotor.set(0.0);
      } 
  
    } else {


      if (xstick.getRawButton(2)) {
        setpoint=setpoint+1; //too fast turn the number down
      } else if (xstick.getRawButton(1)) {
        setpoint=setpoint-1; //this number is not speed 
           } 


      double pid_loop_value = pid.calculate(encoder.get(),setpoint);
      System.out.println("PID output =" + pid_loop_value);
      if (pid_loop_value > 0.35)
      {
        pid_loop_value = 0.35; //this stops the arm from going crazy 
      }
      else if (pid_loop_value < -0.25) {
        pid_loop_value = -0.25;
      
      }
      wristMotor.set(pid_loop_value);
      System.out.println("PID output =" + pid_loop_value);
      System.out.println("sensor=" + encoder.get());


//Limelight values to drive
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    double turningOutput = turningController.calculate(x);
    double movingOutput = movingController.calculate(area - targetArea);

    m_robotDrive.driveCartesian(-turningOutput + movingOutput, movingOutput, turningOutput + movingOutput);


   
}}



    
}

  @Override
  public void disabledInit() {}
  @Override
  public void disabledPeriodic() {}
  @Override
  public void testInit() {}
  @Override
  public void testPeriodic() {}
  @Override
  public void simulationInit() {}
  @Override
  public void simulationPeriodic() {}

  
}
