package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.assembly.PowerPlayBot;

@Disabled
@TeleOp(name = "LaserDistanceTest", group = "Test")
public class TestLaserSensors extends LinearOpMode
{
    //Creating a  robot object
    PowerPlayBot bot = new PowerPlayBot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        bot.initRobot(hardwareMap);

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        while (opModeIsActive())
        {
            /*
            telemetry.addData("Front Laser", bot.getNavigation().frontDistance());
            telemetry.addData("Back Laser", bot.getNavigation().backDistance());
             */
            telemetry.update();
        }
    }
}
