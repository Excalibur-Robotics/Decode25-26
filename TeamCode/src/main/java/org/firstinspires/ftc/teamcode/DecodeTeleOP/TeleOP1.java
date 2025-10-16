package org.firstinspires.ftc.teamcode.DecodeTeleOP;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DECODE.In_take;
import org.firstinspires.ftc.teamcode.DECODE.MovingFWBW;
import org.firstinspires.ftc.teamcode.DECODE.Out_take;
import org.firstinspires.ftc.teamcode.DECODE.turning;

@TeleOp (name= "DECODE1")
public class TeleOP1 extends CommandOpMode {
    GamepadEx gp1;

    Out_take outTake;

    In_take intake;

    MovingFWBW driving;

    turning turn;

    @Override

    public void initialize(){
        CommandScheduler.getInstance().reset();
        gp1 = new GamepadEx(gamepad1);
        outTake = new Out_take(hardwareMap);
        intake = new In_take(hardwareMap);
        driving = new MovingFWBW(hardwareMap);
        turn = new turning(hardwareMap);
    }
    public void run(){
        CommandScheduler.getInstance().run();
        if (gamepad1.a){
            outTake.basicMovement(2);
        }
        else {
            outTake.basicMovement(0);
        }

        if (gamepad1.b){
            intake.movement(-3);
        }
        else {
            intake.movement(0);
        }

        driving.movement(gamepad1.left_stick_y);

        turn.rturn(gamepad1.right_stick_y);
        turn.lturn(-gamepad1.right_stick_y);

    }
}
