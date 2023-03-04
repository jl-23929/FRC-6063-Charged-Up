package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDControl extends PIDController {
    public String name;
    public double value;

    private double[] defaults;
    PIDControl(double p, double i, double d, String name) {
        super(p, i, d);

        double[] vals = { p, i, d };
        this.name = name;
        this.defaults = vals;
        SmartDashboard.putNumberArray(name, vals);
    }

    public void update(double in, double set) {
        double[] vals = SmartDashboard.getNumberArray(name, defaults);
        super.setP(vals[0]);
        super.setI(vals[1]);
        super.setD(vals[2]);

        value = super.calculate(in, set);
    }
}