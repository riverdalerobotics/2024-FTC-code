package org.firstinspires.ftc.teamcode.ThirdRobotCode.DefaultCommands;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    Gamepad gamepad;
    SparkFunOTOS.Pose2D pose2D;
    double xPos,yPos;
    boolean slowMode = false;


    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ChassisDefaultCommand(ChassisSubsystem subsystem, Gamepad gamepad, SparkFunOTOS otos) {
        chassis = subsystem;
        this.otos = otos;
        this.oi= oi;
        this.gamepad = gamepad;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(chassis);
    }
    @Override
    public void initialize(){
        heading = otos.getPosition().h;
    }
    public void execute(){
        pose2D = otos.getPosition();
        xPos = pose2D.x*(-3.048);
        yPos = pose2D.y*(-3.048);
        heading = pose2D.h;
        double speed = gamepad.left_stick_y;
        double strafe = gamepad.left_stick_x;
        double turn = 0.8*gamepad.right_stick_x;
        if(heading<0){
            heading+=360;
        }
        if(gamepad.right_bumper){
            slowMode=true;
        }
        else{
            slowMode = false;
        }
        if(slowMode){
            speed = speed*0.4;
            strafe = strafe*0.4;
            turn = turn*0.4;
        }

        chassis.fieldOriented(heading, -speed, strafe, turn);
    }
    public boolean isFinished(){
        return false;
    }


}
