package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LedSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LedSubsystem() {
        // PWM port 9
      // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(9);

      
      // Reuse buffer
      // Default to a length of 73, start empty output
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(83);
      m_led.setLength(m_ledBuffer.getLength());


      // Write the data to the LED strip
      m_led.start();

      setDefaultCommand(none());
    }

    @Override
    public void periodic() {
      m_led.setData(m_ledBuffer);
    }

    public Command pattern(LEDPattern pattern) {
      return runOnce(() -> pattern.applyTo(m_ledBuffer))
        .andThen(idle())
        .ignoringDisable(true);
    }

    public Command none() {
      return pattern(LEDConstants.kNoBalls);
    }

    public Command oneLeft() {
      return pattern(LEDConstants.kOneLeftBall);
    }

     public Command oneRight() {
      return pattern(LEDConstants.kOneRightBall);
    }

    public Command two() {
      return pattern(LEDConstants.kTwoBalls);
    }
}

