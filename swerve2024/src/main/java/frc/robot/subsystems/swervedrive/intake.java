package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class intake extends SubsystemBase
{
    private final CANSparkMax intakeMotor;

    /**
     * @param intakeMotorID the CAN bus ID
     */
    public intake(int intakeMotorID)
    {
       intakeMotor = new CANSparkMax(intakeMotorID, MotorType.kBrushless);
    }
    /**
     * in robot file set this to the disred button like this OBJname.griper(IWKMS.getR2Button());
     * do this in the teleopPeriodic
     * @param gripers the boolean value of the button
     */
    public void griper(boolean froward, boolean backward)
    {
        if(froward)
        {
            double speed =0.3;
            intakeMotor.set(speed);
        } else if (backward)
        {
            intakeMotor.set(-0.3);
        } else {
            intakeMotor.set(0);
        }
    }
    public Command intakeCommand(boolean direction)
    {
        return Commands.runOnce(()-> griper(direction));
    }
    public void griper(boolean direction)
    {
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis() - startTime;
        long elapsedSeconds = elapsedTime / 1000;
        while(elapsedSeconds>=2)
        {
            elapsedTime = System.currentTimeMillis() - startTime;
            elapsedSeconds = elapsedTime / 1000;
            if(direction)
            {
                double speed =0.3;
                intakeMotor.set(speed);
            }
            else
            {
                intakeMotor.set(-0.3);
            }
        }
    }
    
}