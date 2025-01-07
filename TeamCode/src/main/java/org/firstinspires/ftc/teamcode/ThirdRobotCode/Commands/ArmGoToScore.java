package org.firstinspires.ftc.teamcode.ThirdRobotCode.Commands;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.ArmSubsystem;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

public class ArmGoToScore extends CommandBase {
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
    public ArmGoToScore(ArmSubsystem subsystem, IntakeSubsystem intake, MultipleTelemetry telemetry, PIDFCoefficients pidf) {
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
        armSubsystem.pivotArm(Constants.ArmConstants.BUCKET_ANGLE, 1, pidf);
        intake.pivotIntake(0);
        intake.spinIntake(0);
    }
    @Override
    public void execute(){
        telemetry.addData("MOVING TO SCORE", true);
        armSubsystem.pivotArm(Constants.ArmConstants.BUCKET_ANGLE, 1, pidf);

    }
    public boolean isFinished(){
        return !armSubsystem.isBusy();

    }
    public void end(){
        telemetry.clear();
    }


}
