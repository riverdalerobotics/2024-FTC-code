package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

public class MoveArm extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    ArmSubsystem armSubsystem;
        double armPos;
    MultipleTelemetry telemetry;
    PIDFCoefficients pidf;
    double setPoint;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MoveArm(ArmSubsystem subsystem, MultipleTelemetry telemetry, PIDFCoefficients pidf,  double setPoint) {
        armSubsystem = subsystem;

        this.telemetry = telemetry;
        this.pidf = pidf;
        // Use addRequirements() here to declare subsystem dependencies.
        this.setPoint = setPoint;
        addRequirements(subsystem);

    }
    @Override
    public void initialize(){
        armSubsystem.pivotArm(setPoint, 1, pidf);
    }
    @Override
    public void execute(){
        armSubsystem.pivotArm(setPoint, 1, pidf);
    }
    public boolean isFinished(){
        return !armSubsystem.isBusy();

    }
    public void end(){
        telemetry.clear();
    }


}
