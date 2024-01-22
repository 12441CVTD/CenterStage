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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="WengTeleOp", group="Linear Opmode")

public class WengTeleOp extends LinearOpMode {

    


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor bL = null;
    private DcMotor fR = null;
    private DcMotor bR = null;
    private DcMotor motorA = null;
    private DcMotor motorCarRight = null;
    private DcMotor motorCarLeft = null;
    private Servo gripL = null;
    private Servo gripR = null;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fL = hardwareMap.get(DcMotor.class, "fL");
        bL = hardwareMap.get(DcMotor.class, "bL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bR = hardwareMap.get(DcMotor.class, "bR");
        motorA = hardwareMap.get(DcMotor.class, "motorA");
        motorCarLeft = hardwareMap.get(DcMotor.class, "motorCarLeft");
        motorCarRight = hardwareMap.get(DcMotor.class, "motorCarRight");
        gripL = hardwareMap.get(Servo.class, "gripL");
        gripR = hardwareMap.get(Servo.class, "gripR");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        fL.setDirection(DcMotor.Direction.REVERSE);
        bL.setDirection(DcMotor.Direction.REVERSE);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            // Setup a variable for each drive wheel to save power level for telemetry
            double fLPower;
            double fRPower;
            double bLPower;
            double bRPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            boolean strafeL = gamepad1.dpad_left;
            boolean strafeR = gamepad1.dpad_right;
            double RT = gamepad2.right_trigger;
            double LT = gamepad2.left_trigger;
            double strafe = gamepad1.left_stick_x;
            boolean RB = gamepad2.right_bumper;
            boolean LB = gamepad2.left_bumper;
            boolean xvalue = gamepad1.x;
            boolean yvalue = gamepad1.y;
            boolean bvalue = gamepad2.b;

            double maxTurnSpeed=.5;
            fLPower   = Range.clip(-drive + turn, -maxTurnSpeed, maxTurnSpeed) ;
            fRPower   = Range.clip(-drive - turn, -maxTurnSpeed, maxTurnSpeed) ;
            bRPower   = Range.clip(drive - turn, -maxTurnSpeed, maxTurnSpeed) ;
            bLPower   = Range.clip(drive + turn, -maxTurnSpeed, maxTurnSpeed) ;
            
 

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            if(strafe > 0 || strafeL)
            {
                if (strafeL)
                {
                    strafe = -0.5;
                }
                fL.setPower(strafe);
                bL.setPower(-strafe);
                fR.setPower(-strafe);
                bR.setPower(strafe);
            }
            else if (strafe < 0 || strafeR){
                if (strafeR){
                    strafe = 0.5;
                }
                fL.setPower(strafe);
                bL.setPower(-strafe);
                fR.setPower(-strafe);
                bR.setPower(strafe);
            }
            else{
                double forwardPower = .8;
                fL.setPower(forwardPower*fLPower);
                bL.setPower(forwardPower*bLPower);
                fR.setPower(forwardPower*fRPower);
                bR.setPower(forwardPower*bRPower);  
            }
            
            // this allows us to use both triggers to drive the robot... I think
            //motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (!bvalue){
                //motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                if (LT > 0){
                    //motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motorA.setPower(0.65*LT);
                }
                else if (RT > 0){
                //have to set power to a negative
                    //motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    motorA.setPower(-0.8*RT);
                }
                else
                    //motorA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    motorA.setPower(0);
            }
            else if (bvalue){
                motorA.setPower(-0.35);
            }
                if (RB){
                    gripL.setPosition(1);
                    gripR.setPosition(0);
                }
                
                else if (LB){
                    double closeGripLPosition = 0.4;
                    gripR.setPosition(closeGripLPosition);
                    gripL.setPosition(1-closeGripLPosition);
                }
            if (xvalue){
                motorCarLeft.setPower(0.35);
                motorCarRight.setPower(-0.35);
            }
            else if (yvalue){
                motorCarRight.setPower(0.35);
                motorCarLeft.setPower(-0.35);
            }
            else {
                motorCarRight.setPower(0.0);
                motorCarLeft.setPower(0.0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "fL (%.2f), fR (%.2f), bL (%.2f), bR (%.2f)");
            telemetry.update();
        }
    }
}