package org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ChassisSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.OI;

/**
 * An example command that uses an example subsystem.
 */
public class ChassisDefaultCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    ChassisSubsystem chassis;
    SparkFunOTOS otos;
    double heading;
    OI oi;


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ChassisDefaultCommand(ChassisSubsystem subsystem, OI oi, SparkFunOTOS otos) {
        chassis = subsystem;
        this.otos = otos;
        this.oi= oi;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        heading = otos.getPosition().h;
    }
    public void execute(){
        heading = otos.getPosition().h;
        chassis.fieldOriented(heading, oi.ySpeed(), oi.xSpeed(), oi.turnSpeed());
    }
    public boolean isFinished(){
        return false;
    }


}
