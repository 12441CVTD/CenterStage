package org.firstinspires.ftc.teamcode;///* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="usage ONBOT1, please use", group="Linear OpMode")
public class ContraBassoon extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fL = null;
    private DcMotor fR = null;
    private DcMotor bL = null;
    private DcMotor bR = null;
    private Servo claw = null;
    private Servo flippyL = null;
    private Servo flippyR = null;
    //private Servo turnyL = null;
    private Servo turnyR = null;
    private DcMotor armL = null;
    private DcMotor armR = null;
    private Servo woosh = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        fL  = hardwareMap.get(DcMotor.class, "fL");
        fR = hardwareMap.get(DcMotor.class, "fR");
        bL  = hardwareMap.get(DcMotor.class, "bL");
        bR = hardwareMap.get(DcMotor.class, "bR");
        claw = hardwareMap.get(Servo.class, "claw");
        flippyL = hardwareMap.get(Servo.class, "flippyL");
        flippyR = hardwareMap.get(Servo.class, "flippyR");
        //turnyL = hardwareMap.get(Servo.class, "turnyL");
        turnyR = hardwareMap.get(Servo.class, "turnyR");
        armL = hardwareMap.get(DcMotor.class, "armL");
        armR = hardwareMap.get(DcMotor.class, "armR");
        woosh = hardwareMap.get(Servo.class, "woosh");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fL.setDirection(DcMotor.Direction.FORWARD);
        fR.setDirection(DcMotor.Direction.FORWARD);
        bL.setDirection(DcMotor.Direction.FORWARD);
        bR.setDirection(DcMotor.Direction.FORWARD);
        armL.setDirection(DcMotor.Direction.REVERSE);
        armR.setDirection(DcMotor.Direction.FORWARD);
        fL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        woosh.setPosition(0.3);
        boolean open = true;
        boolean flipped = false;
        boolean turned = false;
        double clawTimerA = -1000;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double fLp;
            double fRp;
            double bLp;
            double bRp;
            double armLP = 0.0;
            double armRP = 0.0;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive =  -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x*0.5;
            double strafe = 0;
            //double strafe = gamepad1.left_stick_x;
            fLp = Range.clip(drive + turn + strafe, -1.0, 1.0);
            fRp = Range.clip(drive - turn - strafe, -1.0, 1.0);
            bLp = Range.clip(drive + turn - strafe, -1.0, 1.0);
            bRp = Range.clip(drive - turn + strafe, -1.0, 1.0);
            if(gamepad1.left_trigger > 0.5 && gamepad1.right_trigger < 0.5){
                // fLp = Range.clip(drive + turn + strafe + 1.0, -1.0, 1.0);
                // fLp = Range.clip(drive + turn - strafe - 1.0, -1.0, 1.0);
                fLp = -0.85;
                fRp = 0.85;
                bLp = 0.85;
                bRp = -0.85;
            }
            else if(gamepad1.left_trigger < 0.5 && gamepad1.right_trigger > 0.5){
                fLp = 0.85;
                fRp = -0.85;
                bLp = -0.85;
                bRp = 0.85;
            }
            if(gamepad1.dpad_up){
                fLp = 0.3;
                fRp = 0.3;
                bLp = 0.3;
                bRp = 0.3;
            }
            else if(gamepad1.dpad_right){
                fLp = 0.3;
                fRp = -0.3;
                bLp = 0.3;
                bRp = -0.3;
            }
            else if(gamepad1.dpad_left){
                fLp = -0.3;
                fRp = 0.3;
                bLp = -0.3;
                bRp = 0.3;
            }
            else if(gamepad1.dpad_down){
                fLp = -0.3;
                fRp = -0.3;
                bLp = -0.3;
                bRp = -0.3;
            }
            if(gamepad2.dpad_up && flipped){
                armLP = 0.5;
                armRP = 0.5;
            }
            else if(gamepad2.dpad_down){
                armLP = -0.5;
                armRP = -0.5;
            }
            if(gamepad2.x && !open){//flipped
                flippyL.setPosition(0.18);//0.2
                flippyR.setPosition(0.82);//0.8
                flipped = true;
                //turnyL.setPosition(0.15);//placing
                turnyR.setPosition(0.8);  //change value to better angle
                turned = true;
            }
            else if(gamepad2.y && !open){//starting position
                flippyL.setPosition(0.88);//0.89
                flippyR.setPosition(0.12);//0.11
                flipped = false;
                //turnyL.setPosition(0.5);//floor level
                turnyR.setPosition(0.51);
                turned = false;
            }
            if(gamepad2.right_bumper){
                if(((runtime.milliseconds() - clawTimerA) > 200)){
                    if(open)
                        claw.setPosition(0.8);//close
                    else if(!open)
                        claw.setPosition(0.45);//open
                    open = !open;
                    clawTimerA = (double) runtime.milliseconds();
                }
            }
            /*if(gamepad2.right_trigger > 0.5){//placing
                turnyL.setPosition(0.15);
                turnyR.setPosition(0.85);
                turned = true;
            }*/
            if(gamepad2.left_trigger > 0.5 && !open){//suspend
                flippyL.setPosition(0.6);
                flippyR.setPosition(0.3);
                turnyR.setPosition(0.3);

            }
            if(gamepad2.a && runtime.seconds() > 120){//airplane, should be 120 seconds for endgame
                woosh.setPosition(0.8);
            }
            if(gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y && gamepad2.a && gamepad2.b && gamepad2.x && gamepad2.y){
                woosh.setPosition(0.8);
                telemetry.addData("Override", "Plane Launch" + 1);
            }


            // Send calculated power to wheels
            fL.setPower(fLp);
            fR.setPower(fRp);
            bL.setPower(bLp);
            bR.setPower(bRp);
            armL.setPower(armLP);
            armR.setPower(armRP);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Status", "ClawA Time: " + clawTimerA);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", armLP, armRP);
            telemetry.update();


        }
    }
}
//
////Odometry code:
////(change stuff ben)
//
//package org.firstinspires.ftc.teamcode.odometry
//
//interface Odometry {
//    fun setAngleRad(angle_rad: Double)
//    fun addAngleBias(angle_rad: Double)
//}
//
////THREE WHEEL ODOMETRY TYPE BEAT (taken from some random place dont even worry about it) SHOWN BELOW:
//
//package org.firstinspires.ftc.teamcode.odometry
//
//import org.firstinspires.ftc.teamcode.field.*
//import org.firstinspires.ftc.teamcode.movement.*
//import org.firstinspires.ftc.teamcode.movement.basicDriveFunctions.*
//
//object ThreeWheel : Odometry{
//        var yTraveled=0.0
//        var xTraveled=0.0
//        var degTraveled=0.0
//
//// last encoder positions
//private var last_l_encoder=0
//private var last_r_encoder=0
//private var last_a_encoder=0
//
//        // used for reading angle absolutely not integrated
//        var angleRadBias=0.0
//
//private var lastRawAngle=0.0
//
//        fun update(curr_l_encoder:Int,curr_r_encoder:Int,curr_a_encoder:Int,leftInchesPerTick:Double,rightInchesPerTick:Double,auxInchesPerTick:Double,turnTrackWidth:Double,auxTrackWidth:Double){
//        DrivePosition.odometer=this
//
//        val lWheelDelta=(curr_l_encoder-last_l_encoder)*leftInchesPerTick
//        val rWheelDelta=(curr_r_encoder-last_r_encoder)*rightInchesPerTick
//        val aWheelDelta=(curr_a_encoder-last_a_encoder)*auxInchesPerTick
//
//
//        // calculate angle change for running arc integration and aux prediction
//        val angleIncrement=(lWheelDelta-rWheelDelta)/turnTrackWidth
//
//        // use absolute for actual angle
//        val leftTotal=curr_l_encoder*rightInchesPerTick
//        val rightTotal=curr_r_encoder*rightInchesPerTick
//        lastRawAngle=((leftTotal-rightTotal)/turnTrackWidth)
//        val finalAngleRad=lastRawAngle+angleRadBias
//
//        // the aux wheel moves when we rotate, so cancel this out with a prediction
//        val aux_prediction=angleIncrement*auxTrackWidth
//
//        val yDelta=(lWheelDelta+rWheelDelta)/2.0
//        val xDelta=aWheelDelta-aux_prediction
//
//        DrivePosition.updatePos(Pose(xDelta,yDelta,angleIncrement),Angle.createUnwrappedRad(finalAngleRad))
//
//        last_l_encoder=curr_l_encoder
//        last_r_encoder=curr_r_encoder
//        last_a_encoder=curr_a_encoder
//        }
//
//        override fun setAngleRad(angle_rad:Double){
//        angleRadBias=angle_rad-lastRawAngle
//        }
//
//        fun addAngleRad(angle_rad:Double){
//        angleRadBias+=angle_rad
//        }
//
//        override fun addAngleBias(angle_rad:Double){
//        angleRadBias+=angle_rad
//        }
//}
//*/
