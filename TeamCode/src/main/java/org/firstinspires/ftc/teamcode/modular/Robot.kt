package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.matrices.GeneralMatrixF
import kotlin.math.absoluteValue
import kotlin.math.sign
import kotlin.reflect.KMutableProperty1

typealias FloatButtonType = KMutableProperty1<Gamepad, Float>
typealias BooleanButtonType = KMutableProperty1<Gamepad, Boolean>

class Robot(private val telemetry: Telemetry) {
    private var speedMultiplier = 1f
    private lateinit var rightFront: DcMotor
    private lateinit var leftBack: DcMotor
    private lateinit var rightBack: DcMotor
    private var leftOdo:Int = leftBack.currentPosition
    private var midOdo:Int = rightFront.currentPosition
    private var rightOdo:Int = rightBack.currentPosition
    private lateinit var drivetrainMotors: Array<DcMotor>
    private lateinit var drivetrainButtons: Array<FloatButton>
    private var currentGamepad1 = Gamepad()
    private var pastGamepad1 = Gamepad()
    private var currentGamepad2 = Gamepad()
    private var pastGamepad2 = Gamepad()
    private val registeredBooleanInputs = HashMap<GamepadButton, (BooleanState) -> Unit>()

    fun initialize(hardwareMap: HardwareMap) {
        val leftFront = hardwareMap.dcMotor["leftFront"]
        val rightFront = hardwareMap.dcMotor["rightFront"]
        val leftBack = hardwareMap.dcMotor["leftRear"]
        val rightBack = hardwareMap.dcMotor["rightRear"]

        this.drivetrainMotors = arrayOf(leftFront, rightFront, leftBack, rightBack)
        // faster than using encoders
        this.drivetrainMotors.forEach { it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER }


        // slows down faster when directional input is let off
        this.drivetrainMotors.forEach { it.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE }

        // go forward on all pos inputs
        leftFront.direction = DcMotorSimple.Direction.REVERSE
        rightFront.direction = DcMotorSimple.Direction.FORWARD
        leftBack.direction = DcMotorSimple.Direction.REVERSE
        rightBack.direction = DcMotorSimple.Direction.FORWARD

        this.telemetry.addLine("Initialized devices")
        this.telemetry.update()
    }

    fun tick(gamepad1: Gamepad, gamepad2: Gamepad) {
        this.updateGamepads(gamepad1, gamepad2)
        this.registeredBooleanInputs.forEach { (button: GamepadButton, consumer: (BooleanState) -> Unit) ->
            this.handleBooleanButtonTick(button as BooleanButton, consumer)
        }
        this.updateDrivetrain()
    }


    private fun updateGamepads(gamepad1: Gamepad, gamepad2: Gamepad) {
        try {
            this.pastGamepad1.copy(this.currentGamepad1)
            this.pastGamepad2.copy(this.currentGamepad2)
            this.currentGamepad1.copy(gamepad1)
            this.currentGamepad2.copy(gamepad2)
        } catch (ignored: Exception) {
        }
    }

    fun registerDrivetrainButtons(
        x: FloatButton,
        y: FloatButton,
        rotateL: FloatButton,
        rotateR: FloatButton
    ) {
        this.drivetrainButtons = arrayOf(x, y, rotateL, rotateR)
    }

    private fun updateDrivetrain() {
        // matrix that holds the directions the wheels need to go
        // for movement in the x, y and rotational axes
        val directionMatrix = GeneralMatrixF(
            4, 3, floatArrayOf(
                1f, -1f, -1f,
                1f, 1f, 1f,
                1f, 1f, -1f,
                1f, -1f, 1f
            )
        )

        // matrix that hold the state of controller input that is translated to wheel speeds
        val inputMatrix = GeneralMatrixF(
            3,
            1,
            this.drivetrainButtons.take(2).map { it.get() }.map { it * -1 }.plus(
                (this.drivetrainButtons[2].get() + -this.drivetrainButtons[3].get()) * this.speedMultiplier.sign
            ).toFloatArray()
        )

        // 4 length float that holds the speeds for each wheel
        // calculated by summing the weights of the wheel movement
        // for each axis via matrix multiplication
        val wheelSpeeds = (directionMatrix.multiplied(inputMatrix) as GeneralMatrixF).data

        // normalize the wheel speeds to within 0..1 and apply the new speeds
        val maxSpeed = wheelSpeeds.maxOfOrNull { it.absoluteValue }!!
        wheelSpeeds.map { if (maxSpeed > 1) it / maxSpeed else it }.zip(this.drivetrainMotors)
            .forEach { it.second.power = it.first.toDouble() * this.speedMultiplier }
    }

    private fun handleBooleanButtonTick(button: BooleanButton, consumer: (BooleanState) -> Unit) {
        val state = this.getState(button)
        consumer(state)
    }

    fun registerButton(button: GamepadButton, consumer: Function1<Robot, Unit>) {
        val wrappingConsumer: (BooleanState) -> Unit = {
            if (it == BooleanState.RISING_EDGE) {
                consumer(this)
            }
        }

        this.registeredBooleanInputs[button] = wrappingConsumer
    }

    fun registerButton(button: GamepadButton, consumer: () -> Unit) {
        val wrappingConsumer: (BooleanState) -> Unit = {
            if (it == BooleanState.RISING_EDGE) {
                consumer()
            }
        }

        this.registeredBooleanInputs[button] = wrappingConsumer
    }

    fun switchDirection() {
        this.speedMultiplier *= -1
    }

    fun quarterSpeed() {
        if (this.speedMultiplier.absoluteValue == 1f) {
            this.speedMultiplier = 0.25f * speedMultiplier.sign
        } else if (this.speedMultiplier.absoluteValue == 0.25f) {
            this.speedMultiplier = 1f * speedMultiplier.sign
        }
    }
    enum class ServoDualState {
        CLOSED,
        OPEN;

        fun opposite(): ServoDualState {
            return when (this) {
                OPEN -> CLOSED
                CLOSED -> OPEN
            }
        }
    }

    enum class BooleanState {
        RISING_EDGE, FALLING_EDGE, INVARIANT
    }

    private fun getState(button: BooleanButton): BooleanState {
        if (button.get() && !button.getPrev()) return BooleanState.RISING_EDGE
        else if (!button.get() && button.getPrev()) return BooleanState.FALLING_EDGE
        return BooleanState.INVARIANT
    }

    fun registerArmButtons(
    ) {
    }

    abstract inner class GamepadButton(private val number: Int) {
        abstract fun get(): Any
        abstract fun getPrev(): Any

        fun getGamepadNum(): Int {
            return this.number
        }

        fun getGamepad(): Gamepad {
            return when (this.number) {
                0 -> this@Robot.currentGamepad1
                1 -> this@Robot.currentGamepad2
                else -> throw IllegalStateException()

            }
        }

        fun getPrevGamepad(): Gamepad {
            return when (this.number) {
                0 -> this@Robot.pastGamepad1
                1 -> this@Robot.pastGamepad2
                else -> throw IllegalStateException()

            }
        }
    }

    inner class FloatButton(private val button: FloatButtonType, number: Int) :
        Robot.GamepadButton(number) {
        override fun get(): Float {
            return this.button.get(this.getGamepad())
        }

        override fun getPrev(): Float {
            return this.button.get(this.getPrevGamepad())
        }
    }

    inner class BooleanButton(private val button: BooleanButtonType, number: Int) :
        Robot.GamepadButton(number) {
        override fun get(): Boolean {
            return this.button.get(this.getGamepad())
        }

        override fun getPrev(): Boolean {
            return this.button.get(this.getPrevGamepad())
        }
    }

    inner class ServoWrapper(
        private val servo: Servo,
        private val open: Double,
        private val closed: Double,
        private var state: ServoDualState = ServoDualState.CLOSED
    ) {
        fun toggle() {
            this.set(this.state.opposite())
        }

        fun set(state: ServoDualState) {
            this.state = state
            when (state) {
                ServoDualState.OPEN -> this.servo.position = this.open
                ServoDualState.CLOSED -> this.servo.position = this.closed
            }
        }

        fun set(position: Double) {
            this.servo.position = position
        }

        fun getPosition(): Double {
            return servo.position
        }
    }
}
