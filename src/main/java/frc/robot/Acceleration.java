package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Acceleration {
  private double accelerationIncrement;
  private double accelerationTime; 

  private Timer motorTimer = new Timer();

  public double motorSpeed = 0;
  public String name;

  
  Acceleration(String name, double accelerationIncrement, double accelerationTime) {
    motorTimer.reset();
    motorTimer.start();

    this.name = name;
    this.accelerationIncrement = accelerationIncrement;
    this.accelerationTime = accelerationTime;
    
    SmartDashboard.putNumber(name+" Speed", motorSpeed);
    SmartDashboard.putNumber(name+" Increment", accelerationIncrement);
    SmartDashboard.putNumber(name+" Time", accelerationTime);
  }
  

  public double set(double desiredSpeed) {
    accelerationIncrement = SmartDashboard.getNumber(name+" Increment", accelerationIncrement);
    accelerationTime = SmartDashboard.getNumber(name+" Time", accelerationTime);


    if (motorTimer.get() >= accelerationTime) {
      accelerationIncrement = (1.5-Math.abs(motorSpeed))*0.6;

      if (Math.abs(desiredSpeed-motorSpeed) <= accelerationIncrement) {
       motorSpeed = desiredSpeed; 
      } else {
        motorSpeed = motorSpeed + accelerationIncrement*Math.signum(desiredSpeed-motorSpeed);
      }
      
      motorTimer.reset();
    }

    SmartDashboard.putNumber(name+" Speed", motorSpeed);
    return motorSpeed;
  }
  
}
