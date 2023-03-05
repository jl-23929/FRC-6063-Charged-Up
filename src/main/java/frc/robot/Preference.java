package frc.robot;

import edu.wpi.first.wpilibj.Preferences;

public class Preference {
    public String key;
    private boolean value;

    Preference(String key, Boolean value) {
        Preferences.initBoolean(key, value);
        this.key = key;
        this.value = value;
    }

    public boolean get() {
        return Preferences.getBoolean(key, value);
    }
}
