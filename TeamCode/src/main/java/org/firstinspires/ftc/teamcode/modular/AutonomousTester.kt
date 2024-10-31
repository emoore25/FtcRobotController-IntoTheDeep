package org.firstinspires.ftc.teamcode.modular


import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Gamepad

@Suppress("unused")
@Autonomous(name = "Autonomous Tester", group = "Prod", preselectTeleOp = "Robot Tester")
class AutonomousTester : LinearOpMode() {
    private var robot = AutonomousRobot(this.telemetry, this::isStopRequested, this::opModeIsActive)

    override fun runOpMode() {
        this.robot.initialize(this.hardwareMap)

        while (!isStarted && !isStopRequested) {
            this.robot.updateGamepads(this.gamepad1, this.gamepad2)
            this.robot.handleButtons()
            this.robot.updateTelemetry()
            this.telemetry.update()
        }
        if (isStarted) {
            this.resetRuntime()
        }
    }
}
