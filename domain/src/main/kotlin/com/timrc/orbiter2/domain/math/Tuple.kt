package com.timrc.orbiter2.domain.math

import com.timrc.orbiter2.domain.enums.Q

/**
 * Faithful Kotlin port of VSE Tuple.java.
 * N-element vector with 1-based indexing.
 */
open class Tuple {
    val N: Int
    val DIM: Int
    protected val vals: DoubleArray
    private var labels: Array<String> = arrayOf("")

    constructor(n: Int) {
        DIM = if (n > 0) n else 1
        N = DIM
        vals = DoubleArray(N)
    }

    constructor(tpl: DoubleArray) : this(tpl.size) {
        tpl.copyInto(vals)
    }

    constructor(tpl: Tuple) : this(tpl.length()) {
        set(tpl)
    }

    constructor(mtx: Matrix) : this(mtx.M) {
        set(mtx)
    }

    open fun getDimension(): Int = DIM
    fun numRows(): Int = N
    fun length(): Int = N

    open fun zero() {
        vals.fill(0.0)
    }

    fun put(i: Int, value: Double) {
        if (i > N || i < 1) throw VectorSpaceIndexOutOfBoundsException("Tuple indices out of bounds: ($i)")
        vals[i - 1] = value
    }

    fun get(i: Int): Double {
        if (i > N || i < 1) throw VectorSpaceIndexOutOfBoundsException("Tuple indices out of bounds: ($i)")
        return vals[i - 1]
    }

    fun set(tup: Tuple) {
        if (tup.N != N) throw VectorSpaceArgumentException("Tuple must be set with a Tuple of $N elements, not: ${tup.N}")
        tup.vals.copyInto(vals)
    }

    fun set(ndx: Int, tup3: Tuple3D) {
        val ndxend = ndx + 2
        if (N < ndxend) throw VectorSpaceIndexOutOfBoundsException("Tuple: index out of bounds: ($ndxend)")
        vals[ndx - 1] = tup3.get(1)
        vals[ndx]     = tup3.get(2)
        vals[ndx + 1] = tup3.get(3)
    }

    fun set(ndx: Int, quat: Quaternion) {
        val ndxend = ndx + 3
        if (N < ndxend) throw VectorSpaceIndexOutOfBoundsException("Tuple: index out of bounds: ($ndxend)")
        vals[ndx - 1] = quat.get(Q.Q0)
        vals[ndx]     = quat.get(Q.QI)
        vals[ndx + 1] = quat.get(Q.QJ)
        vals[ndx + 2] = quat.get(Q.QK)
    }

    fun set(tpl: DoubleArray) {
        if (tpl.size != N) throw VectorSpaceArgumentException("Tuple must be set with an array of $N elements, not: ${tpl.size}")
        tpl.copyInto(vals)
    }

    fun set(mtx: Matrix) {
        if (mtx.N != 1 || mtx.M != N) throw VectorSpaceArgumentException("Tuple must be set with a Matrix of ($N, 1) elements, not: (${mtx.M}, ${mtx.N})")
        val mvals = mtx.valuesPtr()
        for (ii in 0 until N) vals[ii] = mvals[ii][0]
    }

    fun setColumn(colNum: Int, mtx: Matrix) {
        if (N != mtx.M) throw VectorSpaceArgumentException("The Tuple isn't the right size")
        if (colNum > mtx.N || colNum < 1) throw VectorSpaceArgumentException("Invalid column number request")
        val col = colNum - 1
        val mvals = mtx.valuesPtr()
        for (ii in 0 until N) vals[ii] = mvals[ii][col]
    }

    fun values(): DoubleArray = vals.copyOf()

    // Package-internal: returns direct pointer — callers must not mutate carelessly
    internal fun valuesPtr(): DoubleArray = vals

    fun plus(a: Tuple, b: Tuple) {
        if (N != a.N || N != b.N) throw VectorSpaceArgumentException("Tuple.plus(a, b): Dimensions must match.")
        for (ii in 0 until N) vals[ii] = a.vals[ii] + b.vals[ii]
    }

    fun plus(a: Tuple) {
        if (N != a.N) throw VectorSpaceArgumentException("Tuple.plus(a): Dimensions must match.")
        for (ii in 0 until N) vals[ii] += a.vals[ii]
    }

    fun minus(a: Tuple, b: Tuple) {
        if (N != a.N || N != b.N) throw VectorSpaceArgumentException("Tuple.minus(a, b): Dimensions must match.")
        for (ii in 0 until N) vals[ii] = a.vals[ii] - b.vals[ii]
    }

    fun minus(a: Tuple) {
        if (N != a.N) throw VectorSpaceArgumentException("Tuple.minus(a): Dimensions must match.")
        for (ii in 0 until N) vals[ii] -= a.vals[ii]
    }

    open fun mult(s: Double) {
        for (ii in 0 until N) vals[ii] *= s
    }

    open fun div(s: Double) = mult(1.0 / s)

    fun mult(mtr: Matrix, tpl: Tuple) {
        if (mtr.N != tpl.N || mtr.M != N) throw VectorSpaceArgumentException("Dimensions for Matrix * Tuple don't match")
        val tplvals = tpl.valuesPtr()
        val mtrvals = mtr.valuesPtr()
        for (ii in 0 until N) {
            vals[ii] = 0.0
            for (jj in 0 until tpl.N) vals[ii] += mtrvals[ii][jj] * tplvals[jj]
        }
    }

    fun dot(tup: Tuple): Double {
        if (N != tup.N) throw VectorSpaceArgumentException("Dimensions for dot(Tuple) don't match")
        var d = 0.0
        val tupvals = tup.valuesPtr()
        for (ii in 0 until N) d += vals[ii] * tupvals[ii]
        return d
    }

    open fun mag(): Double = Math.sqrt(dot(this))

    open fun unitize() = mult(1.0 / mag())

    open fun setLabels(lbls: Array<String>?) {  // nullable to allow safe override chain
        if (lbls != null) labels = lbls.copyOf()
    }

    open fun getLabels(): Array<String> = labels.copyOf()

    override fun toString(): String = buildString { for (v in vals) append("  $v\n") }
}
