package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase 
{
    private AddressableLED led = new AddressableLED(6);
    private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(30);
    public static final double TRANSITION_LENGTH = 0.4;
}
