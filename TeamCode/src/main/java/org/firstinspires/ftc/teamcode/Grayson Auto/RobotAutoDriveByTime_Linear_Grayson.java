/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareMapFTC;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time", group="Robot")
@Disabled
public class RobotAutoDriveByTime_Linear_Grayson extends LinearOpMode {
    HardwareMapFTC robot = new HardwareMapFTC();




    private ElapsedTime     runtime = new ElapsedTime();


    static final double     FORWARD_SPEED = 0.6;
    static final double     TURN_SPEED    = 0.5;

    public void forward(int time;){
        robot.LFront.setPower(FORWARD_SPEED);
        robot.RFront.setPower(FORWARD_SPEED);
        robot.LBack.setPower(FORWARD_SPEED);
        robot.RBack.setPower(FORWARD_SPEED);
        sleep(time);
        runtime.reset();
    }

    public void right(int time;){
        robot.LFront.setPower(TURN_SPEED);
        robot.RFront.setPower(-TURN_SPEED);
        robot.LBack.setPower(TURN_SPEED);
        robot.RBack.setPower(-TURN_SPEED);
        sleep(time);
        runtime.reset();
    }
    public void left(int time;){
        robot.LFront.setPower(-TURN_SPEED);
        robot.RFront.setPower(TURN_SPEED);
        robot.LBack.setPower(-TURN_SPEED);
        robot.RBack.setPower(TURN_SPEED);
        sleep(time);
        runtime.reset();
    }
    public void backward(int time;){
        robot.RFront.setPower(-FORWARD_SPEED);
        robot.LFront.setPower(-FORWARD_SPEED);
        robot.RBack.setPower(-FORWARD_SPEED);
        robot.LBack.setPower(-FORWARD_SPEED);
        sleep(time);
        runtime.reset();
    }

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.



        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.

        // Step 1:  Drive forward for 3 seconds
        forward(3000);
        backward(3000);
        right(3000);
        forward(3000);
        left(3000);





        /* Step 4:  Stop
        robot.RFront.setPower(0);
        robot.LFront.setPower(0);
        robot.RBack.setPower(0);
        robot.LBack.setPower(0);
          */

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}