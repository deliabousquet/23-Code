/// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//coding is hard

package frc.robot;




import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.*;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   
   *///right motors
 private final VictorSP rightMotor1 = new VictorSP(1);
 private final VictorSP rightMotor2 = new VictorSP(0);



  //left motors
  private final VictorSP leftMotor1 = new VictorSP(3);
  private final VictorSP leftMotor2 = new VictorSP(2);




 
  //arm Controller Motor
  private final VictorSP elbowMotor = new VictorSP(4);
  private final VictorSP wristMotor = new VictorSP (6); 
  private final VictorSP clapMotor = new VictorSP(5);






//Speed Controller Group

  MotorControllerGroup leftSpeedGroup = new MotorControllerGroup(leftMotor1, leftMotor2);
  MotorControllerGroup rightSpeedGroup = new MotorControllerGroup(rightMotor1, rightMotor2);




  //drivetrain




  DifferentialDrive drivetrain = new DifferentialDrive(rightSpeedGroup, leftSpeedGroup);
  
  //joystick
  Joystick stick =new Joystick(0);
  XboxController xstick = new XboxController(1);
  private boolean buttonPressed;








Encoder encoder = new Encoder(0,1);
PIDController pid = new PIDController(0.01, 0, 0);
int setpoint = 0;






  @Override
  public void robotInit() {
    
    buttonPressed = false;
    CameraServer.startAutomaticCapture();




 
  
  }
  
  @Override
  public void robotPeriodic() {}




  @Override
  public void autonomousInit() {




  drivetrain.setSafetyEnabled(false); 




  drivetrain.tankDrive(-0.5,-0.55);
  try{




    //first go forwards




    Thread.sleep(2500);
} catch (InterruptedException e) {
  e.printStackTrace();


}


drivetrain.tankDrive(0.5,0.55);
try{




  //first go back




  Thread.sleep(3000);
} catch (InterruptedException e) {
e.printStackTrace();


}


drivetrain.tankDrive(-0.5,-0.55);
try{




  //first go forwards




  Thread.sleep(4000);
} catch (InterruptedException e) {
e.printStackTrace();


}






drivetrain.tankDrive(0.5,0.55);
try{




  //first go back




  Thread.sleep(4000);
} catch (InterruptedException e) {
e.printStackTrace();


}


  drivetrain.tankDrive(0,0);






  
 




}


  @Override
  public void autonomousPeriodic() {}




  @Override
  public void teleopInit() {


this.encoder.reset();
this.setpoint = 0;


  }




  @Override
  public void teleopPeriodic() {




double driveSpeedScale = -0.9; // Change this value to adjust the drive speed scale








    double forwardSpeed = stick.getY();
    double turnSpeed = stick.getZ();




    // Scale the joystick values to reduce the robot's speed
    forwardSpeed *= driveSpeedScale;
    turnSpeed *= driveSpeedScale;




    // Drive the robot using the scaled joystick values
    drivetrain.arcadeDrive(forwardSpeed, turnSpeed);




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
      //adding a comment hello
      //another comment
      
    }


    




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
