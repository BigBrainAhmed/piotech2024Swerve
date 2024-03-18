package frc.robot.subsystems.swervedrive;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class arm extends SubsystemBase
{
    private final CANSparkMax armMotorA;
    private final CANSparkMax armMotorB;
    private DutyCycleEncoder armEncoder;

    /**
     * @param motorAID the CAN bus ID
     * @param motorBChanel the CAN bus ID
     * @param encoderPort the DIO Port number
     */
    public arm(int motorAID, int motorBID, int encoderPort)
    {
        armMotorA = new CANSparkMax(motorAID, MotorType.kBrushless);
        armMotorB = new CANSparkMax(motorBID,MotorType.kBrushless);
        armEncoder = new DutyCycleEncoder(encoderPort);
        armEncoder.setDistancePerRotation(4.0);
    }
    /**
     * set this to button 4 in robotContainer file in the configureBindings method replace if it alredy exist for that button
     */
    public void presetT()
    {
        angle(0.2, 1, 0.05);
    }
    /**
     * set this to button 3 in robotContainer file in the configureBindings method replace if it alredy exist for that button
     */
    public void presetO()
    {
        angle(0.1, 0.916, 0.02);
    }
    /**
     * set this to button 2 in robotContainer file in the configureBindings method replace if it alredy exist for that button
     */
    
    public void presetX()
    {
        angle(0.1, 0.916, 0.5);
    }
    /**
     * set this to button 1 in robotContainer file in the configureBindings method replace if it alredy exist for that button
     */
    public void presetS()
    {
        angle(0.1, 0, 0.2);
    }
    /**
     * 
     * @param speed Speed the motors turn
     * @param theta the bore encoder value at desired angle
     */
    public void angle(double speed, double theta, double errorbound)
    {
        if (theta - errorbound > armEncoder.getDistance()) {
            armMotorA.set(speed * -1);
            armMotorB.set(speed);
            System.out.println("down");
        } 
        
        if (theta + errorbound < armEncoder.getDistance()) {
            armMotorA.set(speed);
            armMotorB.set(speed * -1);
            System.out.println("up");
        } 
        if(!(theta - errorbound > armEncoder.getDistance() || theta + errorbound < armEncoder.getDistance()))
        {
            armMotorA.set(0);
            armMotorB.set(0);
        }

        //System.out.println("encoder - errorbound" + (armEncoder.getDistance()));
    }
    public Command angleCommand(double speed, double theta, double errorBound) 
    {
        return Commands.runOnce(()-> angle(speed, theta, errorBound,1)); // Assuming 'this' is the subsystem that includes the arm motors and encoder
    }
    public void angle(double speed, double theta, double errorbound,int a)
    {
        if(theta - errorbound > armEncoder.getDistance()) 
        {
            armMotorA.set(speed * -1);
            armMotorB.set(speed);
            while(true)
            {
                if(!(theta - errorbound > armEncoder.getDistance()|| theta + errorbound < armEncoder.getDistance()))
                {
                    armMotorA.set(0);
                    armMotorB.set(0);
                    break;
                }
            }
        } 
        if(theta + errorbound < armEncoder.getDistance()) 
        {
            armMotorA.set(speed * -1);
            armMotorB.set(speed);
            while(true)
            {
                if(!(theta - errorbound > armEncoder.getDistance()|| theta + errorbound < armEncoder.getDistance()))
                {
                    armMotorA.set(0);
                    armMotorB.set(0);
                    break;
                }
            }
        }
    }
    public void manuel(boolean up, boolean down, boolean preSetT, boolean preSetO, boolean preSetX, boolean preSetS){
        if(up){
            armMotorA.set(0.2);
            armMotorB.set(-0.2);
        } else if (down){
            armMotorA.set(-0.2);
            armMotorB.set(0.2);
        } else if (preSetT){
            presetT();
        } else if (preSetO){
            presetO();
        } else if (preSetX){
            presetS();
        } else if (preSetS){
            presetX();
        } else
        {
            armMotorA.set(0);
            armMotorB.set(0);
        }
    }

    public String encoderVal()
    {
        return ""+armEncoder.getDistance();
    }
}
/*
 * okay so assuming you're using pathplanner, somthing like:

public class RobotContainer {
    public Command getAutonomousCommand() {
        
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        
        return AutoBuilder.followPath(path);
    }
}


.fromPathFile() loads a path from a folder (i think the default is the pathplanner folder created by the vendordep inside your robot project)

.followPath is where the main cool stuff happens, it returns a command that follows the path, you can look more into how to create the AutoBuilder here https://pathplanner.dev/pplib-build-an-auto.html


After that, you can make a ui to select an auto any way you want, maybe a drop-down menu in shuffleboard, a seperate program that stitches paths together, whatever you want.

Once I find our implementation i'll link it as well so you can see how we do it 
PathPlanner Docs Help
Build an Auto | PathPlanner Docs

the way we want to do it, which is stupidly overcomplicated and i would not reccomend unless you have loads of time, is kinda like a connect the dots application where the operator/driver can create a path on the fly by connecting points (speaker scoring locations, amp scoring location, note pickup points, etc)


If I were you, I would personally just have a drop down menu with the different paths, especially because you prob wont have too many paths anyways
https://github.com/MontclairRobotics/Crescendo/blob/main/src/main/java/frc/robot/subsystems/Auto.java


Here is our auto implementation, there is a lot of junk code in it for a seperate thing we are working on, but the setupPathPlanner() method is exactly what you're probably looking for 


 */