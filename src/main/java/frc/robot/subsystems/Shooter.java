// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;



public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  private WPI_TalonSRX leftFlyWheel = new WPI_TalonSRX(Constants.leftFlyWheelID);
  private WPI_TalonSRX rightFlyWheel = new WPI_TalonSRX(Constants.rightFlyWheelID);
  private PIDController leftPID = new PIDController(Constants.leftFlyWheelPIDConstants.kp, Constants.leftFlyWheelPIDConstants.ki, Constants.leftFlyWheelPIDConstants.kd, 0.01);
  private PIDController rightPID = new PIDController(Constants.rightFlyWheelPIDConstants.kp, Constants.rightFlyWheelPIDConstants.ki, Constants.rightFlyWheelPIDConstants.kd, 0.01);
  private SimpleMotorFeedforward leftFF = new SimpleMotorFeedforward(Constants.leftFlyWheelFF.kS, Constants.leftFlyWheelFF.kV, Constants.leftFlyWheelFF.kA);
  private SimpleMotorFeedforward rightFF = new SimpleMotorFeedforward(Constants.rightFlyWheelFF.kS, Constants.rightFlyWheelFF.kV, Constants.rightFlyWheelFF.kA);
  
  public Shooter() {
    leftFlyWheel.configFactoryDefault();
    rightFlyWheel.configFactoryDefault();
    
    leftFlyWheel.setInverted(false);
    rightFlyWheel.setInverted(false);

    leftFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    rightFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);


    leftPID.setTolerance(0.05);
    rightPID.setTolerance(0.5);
  }


  public void setSpeed(double goal){
    leftFlyWheel.set(goal);
    rightFlyWheel.set(goal);
  }

  public void setSpeedPID(double goal){
    leftFlyWheel.set(leftFF.calculate(goal)/12 + leftPID.calculate(getAvgRPM(), goal));
    rightFlyWheel.set(rightFF.calculate(goal)/12 + rightPID.calculate(getAvgRPM(), goal));
  }

  public double getEncoderData(){
    return(leftFlyWheel.getSelectedSensorPosition());
  }

  public double getAvgRPM(){
    //Gets ticks over 100ms for both encoders, averages them and converts ticks per 100ms to rotations per minute
    return((leftFlyWheel.getSelectedSensorVelocity()+rightFlyWheel.getSelectedSensorVelocity())*600/Constants.encoderTicks)/2;

  }

  public void resetEncoder(){
    leftFlyWheel.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    if(RobotContainer.getJoystick().getRawButtonPressed(1)){
      setSpeedPID(0.4);
    }
    // This method will be called once per scheduler run
  }
}
