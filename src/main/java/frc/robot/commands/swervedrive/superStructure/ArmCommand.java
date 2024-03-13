package frc.robot.commands.swervedrive.superStructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommand extends Command {
  private final Arm armSubsystem;
  private final double targetPosition;
  private static final double TOLERANCE = 0.1;

  public ArmCommand(Arm armSubsystem, double targetPosition) {
    this.armSubsystem = armSubsystem;
    this.targetPosition = targetPosition;
    addRequirements(armSubsystem);
    System.out.println("armSubsystem" + targetPosition);
  }

  @Override
  public void initialize() {
    armSubsystem.setTargetPosition(targetPosition);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(armSubsystem.getCurrentPosition() - targetPosition) <= TOLERANCE;
  }
}
