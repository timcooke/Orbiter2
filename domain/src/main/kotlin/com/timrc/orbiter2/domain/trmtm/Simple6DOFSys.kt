package com.timrc.orbiter2.domain.trmtm

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.enums.XdX6DQ
import com.timrc.orbiter2.domain.intxm.ISysEqns
import com.timrc.orbiter2.domain.intxm.RK4
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple
import com.timrc.orbiter2.domain.math.Tuple3D
import com.timrc.orbiter2.domain.strtm.MassDyadic

/**
 * Faithful Kotlin port of VSE Simple6DOFSys.java.
 * Abstract base class for a 6DOF rigid-vehicle simulation:
 * 13-element state, quaternion attitude, RK4 integration.
 */
abstract class Simple6DOFSys : ISysEqns, IPosVelAtt {

    companion object {
        const val ORDER = 13
    }

    private var controls: Tuple? = null
    private var ncont: Int = 0
    private var outputs: Tuple? = null
    private var nout: Int = 0

    private var t: Double = 0.0
    private var delta: Double = 0.1
    private val x = State6DQ()

    private val dqs: Simple6DOFdq
    private val integ8r = RK4(ORDER)

    private val jMat = MassDyadic()

    // step() cache
    private val i2bQ = Quaternion()

    init {
        jMat.setMass(1.0)
        jMat.putJ(Basis3D.I, Basis3D.I, 1.0)
        jMat.putJ(Basis3D.J, Basis3D.J, 1.0)
        jMat.putJ(Basis3D.K, Basis3D.K, 1.0)
        x.getAttitude(t, i2bQ)
        dqs = Simple6DOFdq(this)
    }

    override fun getOrder(): Int = ORDER

    fun enableControls(cs: Tuple) { controls = cs; ncont = cs.length() }
    fun disableControls() { ncont = 0 }
    override fun getNumControls(): Int = ncont

    fun enableOutputs(ys: Tuple) { outputs = ys; nout = ys.length() }
    fun disableOutputs() { nout = 0 }
    override fun getNumOutputs(): Int = nout

    override fun getT(): Double = t
    override fun setT(t0: Double) { t = t0 }

    fun getX(i: XdX6DQ): Double = x.get(i)
    fun setX(i: XdX6DQ, x0: Double) { x.put(i, x0) }

    override fun getX(out: Tuple): Double { out.set(x); return t }
    override fun setX(inTuple: Tuple) { x.set(inTuple) }
    override fun getXNames(): Array<String> = x.getLabels()

    override fun getY(out: Tuple) {
        if (nout > 0) out.set(outputs!!) else out.zero()
    }
    override fun getYNames(): Array<String> =
        if (nout > 0) outputs!!.getLabels() else emptyArray()

    override fun getU(out: Tuple) {
        if (ncont > 0) out.set(controls!!) else out.zero()
    }
    override fun setU(inTuple: Tuple) { if (ncont > 0) controls!!.set(inTuple) }
    override fun getUNames(): Array<String> =
        if (ncont > 0) controls!!.getLabels() else emptyArray()

    fun getMass(): Double = jMat.getMass()
    fun setMass(mass: Double) { jMat.setMass(mass) }
    fun getJMat(): MassDyadic = jMat
    fun getJ(row: Basis3D, col: Basis3D): Double = jMat.getJ(row, col)
    fun putJ(row: Basis3D, col: Basis3D, `val`: Double) { jMat.putJ(row, col, `val`) }

    override fun getPosition(tReq: Double, tout: Tuple3D): Double { x.getPosition(t, tout); return t }
    override fun getVelocity(tReq: Double, tout: Tuple3D): Double { x.getVelocity(t, tout); return t }
    override fun getAttitude(tReq: Double, qout: Quaternion): Double { x.getAttitude(t, qout); return t }

    /**
     * Compute all forces, torques, and accelerations for the current state.
     * Called by Simple6DOFdq before each integration stage.
     */
    internal abstract fun finishModel(
        time: Double, state: State6DQ,
        bForce: Tuple3D, bTorque: Tuple3D,
        iForce: Tuple3D, iAccel: Tuple3D
    )

    /** Called after each step; subclass updates output Tuples here. */
    protected abstract fun computeOutputs()

    override fun step() = step(delta)

    override fun step(userDelta: Double) {
        t = integ8r.step(t, userDelta, x, dqs, x)
        x.setTime(t)

        // Normalize quaternion to prevent drift
        x.getAttitude(t, i2bQ)
        i2bQ.normalizeTOL()
        x.setAttitude(i2bQ)

        computeOutputs()
    }
}
