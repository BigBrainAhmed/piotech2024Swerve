package frc.robot.subsystems.swervedrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class cmdShooter extends Command
{
    protected long startTime = System.currentTimeMillis();
    public cmdShooter()
    {
        addRequirements(RobotContainer.Intake);
    }

    @Override
    public void execute() 
    {
        RobotContainer.Shooter.shoot(true,false);
    }

    @Override
    public void initialize() 
    {
        startTime =System.currentTimeMillis();
        RobotContainer.Shooter.shoot(true,false);
    }

    @Override
    public boolean isFinished() 
    {
        
        long elapsedTime = System.currentTimeMillis() - startTime;
        long elapsedSeconds = elapsedTime / 1000;
        return elapsedSeconds>1;
    }
}
