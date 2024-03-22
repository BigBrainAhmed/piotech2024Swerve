package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class shooter extends SubsystemBase
{
    private final CANSparkMax shooterMotorA;
    private final WPI_VictorSPX shooterMotorB;
    /**
     * 
     * @param shooterAID the CAN bus ID
     * @param shooterBChannel device ID of motor controller
     */
    public shooter(int shooterAID,int shooterBChannel)
    {
        shooterMotorA = new CANSparkMax(shooterAID, MotorType.kBrushless);
        shooterMotorB = new WPI_VictorSPX(shooterBChannel);
    }
    /**
     * in robot file set this to the disred button like this OBJname.griper(IWKMS.getR2Button());
     * do this in the teleopPeriodic
     * @param balls the boolean value of the button
     */
    public void shoot(boolean balls,boolean balls2)
    {
        double speed =0.8;
        if(balls)
        {
            
            shooterMotorA.set(speed);
            shooterMotorB.set(speed + 0.2);
        }
        else if(balls2)
        {
            speed*=-1;
            shooterMotorA.set(speed);
            shooterMotorB.set(speed - 0.2);
        }
        else 
        {
            shooterMotorA.set(0);
            shooterMotorB.set(0);
        }
    }
    public Command shootCommand(int seconds)
    {
        return Commands.runOnce(()-> shoot(seconds));
    }
    public void shoot(int seconds)
    {
        //tiemr+ramp time
        long startTime = System.currentTimeMillis();
        long elapsedTime = System.currentTimeMillis() - startTime;
        long elapsedSeconds = elapsedTime / 1000;
        while(elapsedSeconds>=seconds)
        {
            elapsedTime = System.currentTimeMillis() - startTime;
            elapsedSeconds = elapsedTime / 1000;
            double speed =0.8;
            shooterMotorA.set(speed);
            shooterMotorB.set(speed + 0.2);
        }

    }
}