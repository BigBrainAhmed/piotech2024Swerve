package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class cmdArmF extends Command 
{
    protected double theta;
    protected double speed;
    protected double errorbound;
    public cmdArmF(double speed,double theta,double errorbound)
    {
        this.theta = theta;
        this.speed = speed;
        this.errorbound = errorbound;
        addRequirements(RobotContainer.Arm);
    }

    @Override
    public void execute() 
    {
        RobotContainer.Arm.angle(speed,theta,errorbound);
    }

    @Override
    public void initialize() 
    {
        RobotContainer.Arm.angle(speed,theta,errorbound);
    }

    @Override
    public boolean isFinished() 
    {
        return !(theta - errorbound > arm.encoderValu()|| theta + errorbound < arm.encoderValu());
    }
}
