package pid.tester

import pid.ControllerMode
import pid.PID
import pid.ControllerDirection


fun main(args: Array<String>) {
    PidTester().start()
}

class PidTester {
    lateinit var myPID: PID

    //working variables/initial conditions
    var setpoint = 0.0
    var input = 0.0
    var output = 0.0
    var kp = 1.0
    var ki = 2.0
    var kd = 0.0
    var outputStart = 50.0
    var inputStart = 200.0
    var setpointStart = 200.0

    //parameters used to simulate the process connected to the pid
    val ntheta = 50
    var kpmodel = 1.0
    var taup = 50.0
    var theta = DoubleArray(ntheta)
    var integrating = false
    var tindex = 0

    //time variables
    var evalTime = 0L
    var evalInc = 10L
    var serialTime = 0L
    var serialInc = 100L
    var now = 0L

    var limitsMin = 0.0
    var limitsMax = 0.0

    var endPlease = false

    fun start() {
        setup()
        while (!endPlease) {
            loop()
            now += evalInc
        }
    }

    fun setup() {
        myPID = PID({ input }, { output }, { output = it }, { setpoint }, kp, ki, kd, controllerDirection = ControllerDirection.DIRECT, timeFunction = { now })

        //working variables
        input = inputStart
        setpoint = setpointStart
        output = outputStart
        for (i in 0 until ntheta) theta[i] = outputStart

        //initialize pid
        setOutputLimits(-250.0, 250.0)
        myPID.setMode(ControllerMode.AUTOMATIC)


        //initialize serial comm
        println("")
        println("Test Start")

    }

    fun loop() {

        //make sure our evaluations happen at set intervals
        if (now < evalTime) {
            return
        }

        if (now > 60000) {
            println("End Test")
            //block execution to have a clean serial output at the end of the test
            end()
        }

        AlterSimulationConditions()
        SimulateInput()

        myPID.compute()

        if (now >= serialTime)
        {
            serialTime += serialInc
            DoSerial()
        }

        evalTime += evalInc
    }

    private fun AlterSimulationConditions() {
        //setpoint stepper
        if (now > 49000) setpoint = 150.0
        else if (now > 44000) setpoint = 100.0
        else if (now > 38000) setpoint = 500.0
        else if (now > 36000) setpoint = 200.0
        else if (now > 32000)setpoint = 150.0
        else if (now > 20000) setpoint = 200.0
        else if (now > 11000) setpoint = 100.0
        else if (now > 8000) setpoint = 1000.0
        else if (now > 6000) setpoint = 200.0
        else if (now > 2000)setpoint = 150.0
        /*else if(now>3000)setpoint=50;*/

        //limit changes
        if (now > 45000)setOutputLimits(-100.0, 100.0)
        else if (now > 39000)setOutputLimits(0.0, 200.0)
        else if (now > 30000)setOutputLimits(-255.0, 255.0)
        else if (now > 15000)setOutputLimits(-100.0, 100.0)
        else if (now > 9000)setOutputLimits(0.0, 200.0)

        //random mode changes
        if (now > 15000) myPID.setMode(ControllerMode.AUTOMATIC)
        else if (now > 10900) myPID.setMode(ControllerMode.MANUAL)
        else if (now > 8500) myPID.setMode(ControllerMode.AUTOMATIC)
        else if (now > 6800) myPID.setMode(ControllerMode.MANUAL)
        else if (now > 4500) myPID.setMode(ControllerMode.AUTOMATIC)
        else if (now > 4000) myPID.setMode(ControllerMode.AUTOMATIC)

        //tunings changes
        if (now > 43000) myPID.setTunings(3.0, .15, 0.15)
        else if (now > 39000) myPID.setTunings(.5, .1, .05)
        else if (now > 30000) myPID.setTunings(0.1, .05, 0.0)
        else if (now > 13000) myPID.setTunings(0.5, 2.0, 0.15)
        else if (now > 9000) myPID.setTunings(2.0, 1.0, .05)

        //model change: switch the nature of the process connected to the pid
        integrating = (now >= 30000)
    }

    private fun SimulateInput() {
        //Create a dead time by using a circular buffer
        theta[tindex] = output
        tindex++
        if (tindex >= ntheta) tindex = 0

        // Compute the input
        if (integrating)
            input = kpmodel / taup * (theta[tindex] - outputStart) + input
        else
            input = kpmodel / taup * (theta[tindex] - outputStart) + (input - inputStart) * (1 - 1 / taup) + inputStart

        //add some noise
        //input += random(-10.0, 10.0) / 100
    }

    private fun DoSerial() {
        println("""$now
            | Kp ${myPID.getKp().toString(2)}
            | Ki ${myPID.getKi().toString(2)}
            | Kd ${myPID.getKd().toString(2)}
            | ${if (myPID.getMode() == ControllerMode.AUTOMATIC) "A" else "M"}
            | limits (${limitsMin.toString(2)}, ${limitsMax.toString(2)}),
            | setpoint ${setpoint.toString(2)}
            | input ${input.toString(2)}
            | output ${output.toString(2)}""".trimMargin().replace("\n", ""))
    }

    fun setOutputLimits(min: Double, max: Double) {
        limitsMin = min
        limitsMax = max
        myPID.setOutputLimits(min, max)
    }

    private fun random(lowerBound: Double, upperBound: Double): Double {
        return Math.random() * (upperBound - lowerBound) + lowerBound
    }

    private fun end() {
        endPlease = true
    }

    private fun Double.toString(precision: Int): String {
        return String.format("%.${precision}f", this)
    }
}
