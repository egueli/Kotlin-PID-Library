package pid

/**
 * PID library, ported from [https://github.com/br3ttb/Arduino-PID-Library].
 */
class PID
@JvmOverloads constructor(
        private val inputCallback: () -> Double,
        private val outputCallback: (Double) -> Unit,
        private val setpointCallback: () -> Double,
        Kp: Double,
        Ki: Double,
        Kd: Double,
        private var proportionalOn: ProportionalOn = ProportionalOn.ERROR,
        controllerDirection: ControllerDirection
) {
    private var inAuto: Boolean = false
    private var lastTime: Long = 0
    private var sampleTime: Long = 100
    private var lastInput: Double = 0.0
    private var lastOutput: Double = 0.0
    private var outputSum: Double = 0.0
    private var kp: Double = 0.0
    private var ki: Double = 0.0
    private var kd: Double = 0.0
    private lateinit var controllerDirection: ControllerDirection
    private var outMax: Double = 0.0
    private var outMin: Double = 0.0
    private var dispKp: Double = 0.0
    private var dispKi: Double = 0.0
    private var dispKd: Double = 0.0

    init {
        setOutputLimits(0.0, 255.0)
        setControllerDirection(controllerDirection)
        setTunings(Kp, Ki, Kd, proportionalOn)
        lastTime = millis() - sampleTime
    }

    fun compute(): Boolean {
        if (!inAuto) return false

        val now = millis()
        val timeChange = now - lastTime
        if (timeChange < sampleTime) return false

        /*Compute all the working error variables*/
        val input = inputCallback.invoke()
        val error = setpointCallback.invoke() - input
        val dInput = input - lastInput
        outputSum += ki * error

        /*Add Proportional on Measurement, if P_ON_M is specified*/
        if (proportionalOn == ProportionalOn.ERROR) {
            outputSum -= kp * dInput
        }

        if (outputSum > outMax) outputSum = outMax
        if (outputSum < outMin) outputSum = outMin

        /*Add Proportional on Error, if P_ON_E is specified*/
        var output = if (proportionalOn == ProportionalOn.ERROR) {
            kp * error
        } else {
            0.0
        }

        /*Compute Rest of PID Output*/
        output += outputSum - kd * dInput

        if (output > outMax) output = outMax
        if (output < outMin) output = outMin
        outputCallback.invoke(output)
        lastOutput = output

        /*Remember some variables for next time*/
        lastInput = input
        lastTime = now
        return true
    }

    /**
     * This function allows the controller's dynamic performance to be adjusted.
     * it's called automatically from the constructor, but tunings can also
     * be adjusted on the fly during normal operation.
     */
    @JvmOverloads
    fun setTunings(Kp: Double, Ki: Double, Kd: Double, proportionalOn: ProportionalOn = this.proportionalOn) {
        if (Kp < 0 || Ki < 0 || Kd < 0) throw IllegalArgumentException("Kp, Ki and Kd must be non-negative")

        this.proportionalOn = proportionalOn

        dispKp = Kp
        dispKi = Ki
        dispKd = Kd

        val sampleTimeInSec = sampleTime / 1000.0
        kp = Kp
        ki = Ki * sampleTimeInSec
        kd = Kd / sampleTimeInSec

        if (controllerDirection == ControllerDirection.REVERSE) {
            kp = -kp
            ki = -ki
            kd = -kd
        }
    }

    /**
     * sets the period, in Milliseconds, at which the calculation is performed
     */
    fun setSampleTime(newSampleTime: Long) {
        if (newSampleTime <= 0) throw IllegalArgumentException("sample time must be higher than zero")

        val ratio = newSampleTime.toDouble() / sampleTime
        ki *= ratio
        kd /= ratio
        sampleTime = newSampleTime
    }

    /**
     *  This function will be used far more often than SetInputLimits.  while
     *  the input to the controller will generally be in the 0-1023 range (which is
     *  the default already,)  the output will be a little different.  maybe they'll
     *  be doing a time window and will need 0-8000 or something.  or maybe they'll
     *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
     *  here.
     */
    fun setOutputLimits(min: Double, max: Double) {
        if (min >= max) throw IllegalArgumentException("maximum output must be higher than the minimum output")

        outMin = min
        outMax = max

        if (inAuto) {
            if (lastOutput > outMax) outputCallback.invoke(outMax)
            else if (lastOutput < outMin) outputCallback.invoke(outMin)

            if (outputSum > outMax) outputSum = outMax
            else if (outputSum < outMin) outputSum = outMin
        }
    }

    /**
     * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
     * when the transition from manual to auto occurs, the controller is
     * automatically initialized
     */
    fun setMode(mode: ControllerMode)
    {
        val newAuto = (mode == ControllerMode.AUTOMATIC);
        if(newAuto && !inAuto)
        {  /*we just went from manual to auto*/
            initialize();
        }
        inAuto = newAuto;
    }

    fun initialize() {
        outputSum = lastOutput
        lastInput = inputCallback.invoke()
        if (outputSum > outMax) outputSum = outMax
        else if (outputSum < outMin) outputSum = outMin
    }

    /**
     * The PID will either be connected to a DIRECT acting process (+Output leads
     * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
     * know which one, because otherwise we may increase the output when we should
     * be decreasing.  This is called from the constructor.
     */
    fun setControllerDirection(direction: ControllerDirection) {
        if (inAuto && direction != controllerDirection) {
            kp = -kp
            ki = -ki
            kd = -kd
        }
        this.controllerDirection = direction
    }

    fun getKp() = dispKp
    fun getKi() = dispKi
    fun getKd() = dispKd
    fun getMode() = if (inAuto) ControllerMode.AUTOMATIC else ControllerMode.MANUAL

    private fun millis(): Long = System.currentTimeMillis()
}

enum class ProportionalOn { MEASUREMENT, ERROR }
enum class ControllerMode { MANUAL, AUTOMATIC }
enum class ControllerDirection { DIRECT, REVERSE }