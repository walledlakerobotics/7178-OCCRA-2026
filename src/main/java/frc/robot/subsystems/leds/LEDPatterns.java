package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public enum LEDPatterns {
    None(LEDPattern.solid(Color.kBlack)),
    One(LEDPattern.solid(Color.kWhite)),
    Two(LEDPattern.solid(Color.kDarkBlue));
    

    LEDPattern m_Pattern;

    LEDPatterns(LEDPattern pattern) {
        m_Pattern = pattern;
    }

    public LEDPattern getPattern() {
        return m_Pattern;
    }
}
