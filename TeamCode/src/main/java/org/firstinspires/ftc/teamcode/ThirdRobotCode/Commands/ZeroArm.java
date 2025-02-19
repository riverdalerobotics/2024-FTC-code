package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

public class ZeroArm extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    ArmSubsystem armSubsystem;
    IntakeSubsystem intake;
    double armPos;
    MultipleTelemetry telemetry;
    PIDFCoefficients pidf;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ZeroArm(ArmSubsystem subsystem, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients pidf) {
        armSubsystem = subsystem;
        this.intake = intake;
        this.telemetry = telemetry;
        this.pidf = pidf;
        // Use addRequirements() here to declare subsystem dependencies.

        addRequirements(subsystem);
        addRequirements(intake);
    }
    @Override
    public void initialize(){
        armSubsystem.pivotArm(0, 1, pidf);
        intake.pivotIntake(0);
        intake.spinIntake(0);
    }
    @Override
    public void execute(){
        telemetry.addData("ZEROING", true);
        armSubsystem.pivotArm(0, 1, pidf);

    }
    public boolean isFinished(){
        return !armSubsystem.isBusy();

    }
    public void end(){
        telemetry.clear();
        armSubsystem.pivotArm(0,0,pidf);
    }


}
