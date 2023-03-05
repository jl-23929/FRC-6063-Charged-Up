package frc.robot;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class MaxPID implements Sendable {
    private static int instances;
    public SparkMaxPIDController controller;

    MaxPID(SparkMaxPIDController controller) {
        this.controller = controller;

        instances++;
        SendableRegistry.addLW(this, "PIDController", instances);
    }

    private double setpoint = 0;
    public void setSetpoint(double point) {
        this.controller.setReference(point, ControlType.kPosition);
        setpoint = point;
    }
    private double getSetpoint() {
        return setpoint;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("PIDController");
      builder.addDoubleProperty("p", this.controller::getP, this.controller::setP);
      builder.addDoubleProperty("i", this.controller::getI, this.controller::setI);
      builder.addDoubleProperty("d", this.controller::getD, this.controller::setD);
      builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    }
}
