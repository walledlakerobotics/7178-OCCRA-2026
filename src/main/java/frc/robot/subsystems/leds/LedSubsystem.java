package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;


public class LedSubsystem extends SubsystemBase {
    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;
    AddressableLEDBufferView m_leftLED;
    AddressableLEDBufferView m_rightLED;
    AddressableLEDBufferView m_leftRightLED;
    AddressableLEDBufferView m_signLED;




    public LedSubsystem() {
        // PWM port 9
      // Must be a PWM header, not MXP or DIO
      m_led = new AddressableLED(9);

      
      // Reuse buffer
      // Default to a length of 73, start empty output
      // Length is expensive to set, so only set it once, then just update data
      m_ledBuffer = new AddressableLEDBuffer(135); //83 without LED Sign
      m_leftRightLED = m_ledBuffer.createView(0, 82);
      m_leftLED = m_ledBuffer.createView(0, 41);
      m_rightLED = m_ledBuffer.createView(42, 82);
      m_signLED = m_ledBuffer.createView(83, 134);
      m_led.setLength(m_ledBuffer.getLength());

      // Write the data to the LED strip
      m_led.start();

      setDefaultCommand(none());
    }

    @Override
    public void periodic() {
     m_led.setData(m_ledBuffer);
     
    }

    public Command pattern(LEDPattern pattern, AddressableLEDBufferView buffer) {
      return runOnce(() -> pattern.applyTo(buffer))
        .andThen(idle())
        .ignoringDisable(true);
    }

    public Command pattern(LEDPattern pattern, AddressableLEDBuffer buffer) {
      return runOnce(() -> pattern.applyTo(buffer))
        .andThen(idle())
        .ignoringDisable(true);
    }

    public Command none() {
      return pattern(LEDConstants.kRed, m_leftRightLED);
    }

    public Command oneLeft() {
      return pattern(LEDConstants.kGreen, m_leftLED);
      //.andThpattern(LEDConstants.kBlue, m_signLED));
    }

     public Command oneRight() {
      return pattern(LEDConstants.kGreen, m_rightLED);
      //.andThen(pattern(LEDConstants.kBlue, m_signLED));
    }

    public Command two() {
       return pattern(LEDConstants.kBlue, m_ledBuffer);
     
    }
}

