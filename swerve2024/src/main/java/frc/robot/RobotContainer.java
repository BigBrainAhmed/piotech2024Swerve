// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.arm;
import frc.robot.subsystems.swervedrive.intake;
import frc.robot.subsystems.swervedrive.shooter;

//import frc.robot.subsystems.swervedrive.test;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final SendableChooser<Command> autoChooser;
  intake Intake = new intake(11);
  shooter Shooter = new shooter(12, 13);
  arm Arm = new arm(9, 10, 0);
  //shooter 1 12
  //shooter 2 13
  //intake 11
  

  //test a = new test(9,10); 
  // The robot's subsystems and commands are defined here...
  private final static SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),"swerve/neo"));
  // CommandJoystick rotationController = new CommandJoystick(1);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandJoystick driverController = new CommandJoystick(1);

  // CommandJoystick driverController   = new CommandJoystick(3);//(OperatorConstants.DRIVER_CONTROLLER_PORT);
  PS4Controller driverPS4 = new PS4Controller(0);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("angle", Arm.angleCommand(0, 0, 0));
    NamedCommands.registerCommand("b",Arm.angleCommand(0, 0, 0));
    NamedCommands.registerCommand("c",Arm.angleCommand(0, 0, 0));
    NamedCommands.registerCommand("inttake",Intake.intakeCommand(true));
    //NamedCommands.registerCommand("shot",Shotter.shotterCommand(#param));
//intakeCommand

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    double sppedmult = 1;
    // Configure the trigger bindings
    configureBindings();

    /*AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverPS4.getLeftY()*sppedmult,
                                                                                            OperatorConstants.LEFT_Y_DEADBAND),
                                                            () -> MathUtil.applyDeadband(driverPS4.getLeftX()*sppedmult,
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverPS4.getRightX()*-1,
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                    driverPS4::getTriangleButtonPressed,
                                                                    driverPS4::getCrossButtonPressed,
                                                                    driverPS4::getSquareButtonPressed,
                                                                    driverPS4::getCircleButtonPressed);*/

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation 
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY()*sppedmult, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX()*sppedmult, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRightX()*-1,
        () -> driverPS4.getRightY()*-1);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    /*Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY()*sppedmult, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX()*sppedmult, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRightX() * -0.5);*/

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(driverPS4.getLeftY()*sppedmult, OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(driverPS4.getLeftX()*sppedmult, OperatorConstants.LEFT_X_DEADBAND),
        () -> driverPS4.getRawAxis(3));

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandPS4Controller PS4}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
  /** Represents an axis on an XboxController. 
    public enum Axis {
    /** Left X. 
    kLeftX(0),
    /** Right X. 
    kRightX(4),
    /** Left Y. 
    kLeftY(1),
    /** Right Y. 
    kRightY(5),
    /** Left trigger. 
    kLeftTrigger(2),
    /** Right trigger. 
    kRightTrigger(3);

    /** Axis value. 
    public final int value;

    Axis(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the axis, matching the relevant methods. This is done by
     * stripping the leading `k`, and if a trigger axis append `Axis`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the axis.
  
    @Override
    public String toString() {
      var name = this.name().substring(1); // Remove leading `k`
      if (name.endsWith("Trigger")) {
        return name + "Axis";
      }
      return name;
    }
  }
  
X: 4.577706567943096E-5
Y: -1.0
X: 4.577706567943096E-5
Y: -1.0
X: 1.5259021893143654E-5
Y: -1.0
X: -0.046860456466674805
Y: -1.0
X: -0.046860456466674805
Y: -1.0
X: -0.046860456466674805
Y: -1.0
X: -0.046860456466674805
Y: -1.0
X: -0.046860456466674805
Y: -1.0
X: -0.046860456466674805
Y: -1.0
X: -0.039047837257385254
Y: -1.0
X: 0.0317845419049263
Y: -1.0
X: 0.11909666657447815
Y: -1.0
  
  
  
  
  
  
  
  
  */