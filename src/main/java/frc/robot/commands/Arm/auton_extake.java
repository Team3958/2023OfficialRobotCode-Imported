package frc.robot.commands.Arm;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake;

public class auton_extake extends CommandBase {
  intake intake;
  static double startTIme;

  int count = 0;
  /** Creates a new Extake. */
  public auton_extake(intake i) {

    intake = i;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTIme = Timer.getMatchTime();
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.Intaking(0.40);
    count +=1;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.Intaking(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   if (count == 50){
      return true;
    }
    return false;
  }
}
