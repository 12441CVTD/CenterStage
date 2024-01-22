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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="SEASON 2023 PLEASE START USING THIS ONE", group="Linear Opmode")

public class Season2023TeleOp2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fR = null;
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private DcMotor arm = null;
    private Servo sR = null;
    private Servo sL = null;
    private Servo Turny = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fR = hardwareMap.get(DcMotor.class, "fR");
        fL = hardwareMap.get(DcMotor.class, "fL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        bL = hardwareMap.get(DcMotor.class, "bL");
        arm = hardwareMap.get(DcMotor.class, "arm");
        sL = hardwareMap.get(Servo.class, "sL");
        sR = hardwareMap.get(Servo.class, "sR");
        Turny = hardwareMap.get(Servo.class, "Turny");
        

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fR.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);
        bR.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double fLP;
            double fRP;
            double bLP;
            double bRP;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  0.8 * gamepad1.right_stick_x;
            int pos = 0;
            boolean active = false;
            
            if(gamepad1.x){
                fL.setPower(-0.85);
                fR.setPower(0.85);
                bL.setPower(0.95);
                bR.setPower(-0.95);
            }
            if(gamepad1.y){
                fL.setPower(0.85);
                fR.setPower(-0.85);
                bL.setPower(-0.95);
                bR.setPower(0.95);
            } else {
                fL.setPower(0.85 * Range.clip(drive + turn, -1.0, 1.0));
                fR.setPower(0.85 * Range.clip(drive - turn, -1.0, 1.0));
                bL.setPower(0.85 * Range.clip(drive + turn, -1.0, 1.0));
                bR.setPower(0.85 *Range.clip(drive - turn, -1.0, 1.0));
            }
            if(gamepad2.right_bumper){ //open
                sL.setPosition(0.61);//left and right are from robot pov
                sR.setPosition(0.32);
            }
            if(gamepad2.left_bumper){ //close
                sL.setPosition(0.45);//purpur
                sR.setPosition(0.48);//orangge
            }
            if(gamepad1.dpad_up){
                fL.setPower(0.45);
                fR.setPower(0.45);
                bL.setPower(0.45);
                bR.setPower(0.45);
            }
            if(gamepad1.dpad_left){
                fL.setPower(-0.45);
                fR.setPower(0.45);
                bL.setPower(-0.45);
                bR.setPower(0.45);
            }
            if(gamepad1.dpad_right){
                fL.setPower(0.45);
                fR.setPower(-0.45);
                bL.setPower(0.45);
                bR.setPower(-0.45);
            }
            if(gamepad2.dpad_up)
                arm.setPower(0.8);
            else if(gamepad2.dpad_down)
                arm.setPower(-0.6);
            else
                arm.setPower(0.05);
            if(gamepad2.x)
                Turny.setPosition(0.235);
            if(gamepad2.y)
                Turny.setPosition(0.05);
            if(gamepad2.a)
                Turny.setPosition(0.35);
            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.update();
        }
    }
}
