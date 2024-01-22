package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous

public class HenryAuto extends LinearOpMode
{
   // Declare OpMode members.
   private ElapsedTime eTime = new ElapsedTime();

   private DcMotor[] motors = new DcMotor[4];
   private String[] motorNames = {
           "fL",
           "fR",
           "bL",
           "bR",
           };

   @Override
   public void runOpMode()
   {

       for(int i=0; i<motors.length; i++) {
           motors[i] = hardwareMap.get(DcMotor.class, motorNames[i]);

           if (i % 2 == 1 && i<4)
               motors[i].setDirection(DcMotor.Direction.REVERSE);
           else
               motors[i].setDirection(DcMotor.Direction.FORWARD);
       }

       telemetry.addData("Status", "Waiting_For_Start" );
       telemetry.update();

       // Wait for the game to start (driver presses PLAY)
       waitForStart();
       eTime.reset();


       // First step of Autonomous (while the program is active and time is less than 2
       while(opModeIsActive() && eTime.seconds() < 1.8)
       {
           for(int i = 0; i<4;i++){
               motors[i].setPower(0.5);
           }

           telemetry.addData("DC Motor ", "RunTime: %2.2f S Elapsed", eTime.seconds() );
           telemetry.update();

       }//closes while loop

   }//closes runOpMode method

}//closes class

