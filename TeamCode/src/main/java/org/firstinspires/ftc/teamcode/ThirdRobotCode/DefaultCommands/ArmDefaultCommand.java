package org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;

/**
 * An example command that uses an example subsystem.
 */
public class ArmDefaultCommand extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    ArmSubsystem armSubsystem;
    double armPos;
    Telemetry telemetry;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ArmDefaultCommand(ArmSubsystem subsystem, Telemetry telemetry) {
        armSubsystem = subsystem;
        this.telemetry = telemetry;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public void initialize(){
        armPos = armSubsystem.getPos();
    }
    public void execute(){

       // armSubsystem.pivotArm(armSubsystem.getPos(), Constants.ArmConstants.REST_SPEED, Constants.ArmConstants.armPID);
        telemetry.addData("IS RUNNING", true);
        if(armSubsystem.getPos()<6){
            armSubsystem.stopArm();
            telemetry.addData("THE POWER SHOULD BE ZERO", true);
        }
        else{
            armSubsystem.pivotArm(armSubsystem.getPos(), Constants.ArmConstants.REST_SPEED, Constants.ArmConstants.armPID);
        }
    }
    @Override
    public boolean isFinished(){
        return false;
    }


}