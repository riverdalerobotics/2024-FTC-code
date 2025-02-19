package org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.Constants;
import org.firstinspires.ftc.teamcode.ThirdRobotCode.IntakeSubsystem;

public class IntakeDefaultCommand extends CommandBase {
    IntakeSubsystem intake;
    Telemetry telemetry;
    public IntakeDefaultCommand(IntakeSubsystem intake, Telemetry telemetry){
        this.intake = intake;
        this.telemetry = telemetry;
        addRequirements(intake);
    }
    public void initialize(){
        intake.spinIntake(0);
        telemetry.addLine("hello");
        telemetry.update();
    }
}
