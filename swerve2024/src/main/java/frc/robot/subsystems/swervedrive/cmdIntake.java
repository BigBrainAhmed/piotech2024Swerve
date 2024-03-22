package frc.robot.subsystems.swervedrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;


public class cmdIntake extends Command
{
    protected long startTime = System.currentTimeMillis();
    public cmdIntake()
    {
        addRequirements(RobotContainer.Shooter);
    }

    @Override
    public void execute() 
    {
        RobotContainer.Intake.griper(true,false);
    }

    @Override
    public void initialize() 
    {
        startTime =System.currentTimeMillis();
        RobotContainer.Intake.griper(true,false);
    }

    @Override
    public boolean isFinished() 
    {
        
        long elapsedTime = System.currentTimeMillis() - startTime;
        long elapsedSeconds = elapsedTime / 1000;
        return elapsedSeconds>1;
    }
}
