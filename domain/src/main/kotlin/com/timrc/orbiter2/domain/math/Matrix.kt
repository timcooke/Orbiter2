package com.timrc.orbiter2.domain.math

/**
 * Faithful Kotlin port of VSE Matrix.java.
 * MxN matrix with 1-based indexing.
 */
open class Matrix {
    companion object {
        const val EPS = 1.0e-100
    }

    val DIM: Int
    val M: Int
    val N: Int
    private val mxm: Boolean
    private var recursiveMat = false
    private val vals: Array<DoubleArray>
    private var row_order: IntArray? = null
    private var row_scale: DoubleArray? = null

    constructor(m: Int, n: Int) {
        M = if (m > 0) m else 1
        N = if (n > 0) n else 1
        DIM = M * N
        mxm = M == N
        vals = Array(M) { DoubleArray(N) }
    }

    constructor(m: Int) : this(m, m)

    constructor(mtrx: Matrix) : this(mtrx.M, mtrx.N) { set(mtrx) }

    constructor(mtrx: Array<DoubleArray>) : this(mtrx.size, mtrx[0].size) {
        for (ii in 0 until M) mtrx[ii].copyInto(vals[ii])
    }

    // Package-internal: for recursive determinant computation
    internal constructor(tmpMtx: Array<DoubleArray>, tmpFlag: Boolean) {
        M = tmpMtx.size; N = tmpMtx[0].size; DIM = M * N; mxm = M == N
        recursiveMat = tmpFlag; vals = tmpMtx
    }

    fun getDimension(): Int = DIM
    fun numRows(): Int = M
    fun numCols(): Int = N
    fun isSquare(): Boolean = mxm

    open fun zero() { for (row in vals) row.fill(0.0) }

    fun zeroUpper() {
        for (ii in 0 until M - 1)
            for (jj in N - 1 downTo ii + 1) vals[ii][jj] = 0.0
    }

    open fun identity() {
        if (!mxm) throw VectorSpaceArgumentException("Not a square matrix - can't create an identity matrix")
        zero()
        for (ii in 0 until M) vals[ii][ii] = 1.0
    }

    fun put(i: Int, j: Int, value: Double) {
        if (i > M || j > N || i < 1 || j < 1)
            throw VectorSpaceIndexOutOfBoundsException("Matrix indices out of bounds: ($i,$j)")
        vals[i - 1][j - 1] = value
    }

    fun get(i: Int, j: Int): Double {
        if (i > M || j > N || i < 1 || j < 1)
            throw VectorSpaceIndexOutOfBoundsException("Matrix indices out of bounds: ($i,$j)")
        return vals[i - 1][j - 1]
    }

    fun set(mtrx: Matrix) {
        if (mtrx.M != M || mtrx.N != N)
            throw VectorSpaceArgumentException("Matrix must be set with a Matrix of ($M, $N) elements, not: (${mtrx.M}, ${mtrx.N})")
        val mv = mtrx.valuesPtr()
        for (ii in 0 until M) mv[ii].copyInto(vals[ii])
    }

    fun set(row: Int, col: Int, mtrx: Matrix) {
        if ((M + 1 - row) < mtrx.M || (N + 1 - col) < mtrx.N)
            throw VectorSpaceArgumentException("Sub Matrix must be (${M+1-row}, ${N+1-col}) or smaller.")
        for (ii in 0 until mtrx.M)
            for (jj in 0 until mtrx.N) put(row + ii, col + jj, mtrx.get(ii + 1, jj + 1))
    }

    fun setColumn(colNum: Int, tpl: Tuple) {
        if (tpl.N != M) throw VectorSpaceArgumentException("Your Tuple isn't big enough")
        if (colNum > N || colNum < 1) throw VectorSpaceArgumentException("Invalid column number request")
        val col = colNum - 1
        val tv = tpl.valuesPtr()
        for (ii in 0 until M) vals[ii][col] = tv[ii]
    }

    fun set(mtrx: Array<DoubleArray>) {
        if (mtrx.size != M || mtrx[0].size != N)
            throw VectorSpaceArgumentException("Matrix must be set with a double[][] of ($M, $N) elements, not: (${mtrx.size}, ${mtrx[0].size})")
        for (ii in 0 until M) mtrx[ii].copyInto(vals[ii])
    }

    fun set(tpl: Tuple) {
        if (N != 1 || tpl.N != M)
            throw VectorSpaceArgumentException("Matrix must be set with a Matrix of ($M, $N) elements, not: (${tpl.N}, 1)")
        val tv = tpl.valuesPtr()
        for (ii in 0 until M) vals[ii][0] = tv[ii]
    }

    fun values(): Array<DoubleArray> = Array(M) { ii -> vals[ii].copyOf() }

    // Package-internal: returns direct pointer — callers must not mutate carelessly
    internal fun valuesPtr(): Array<DoubleArray> = vals

    fun plus(aMat: Matrix) {
        if (M != aMat.M || N != aMat.N)
            throw VectorSpaceArgumentException("Wrong dimensions for Matrix.plus(Matrix aMat)")
        val av = aMat.valuesPtr()
        for (ii in 0 until M) for (jj in 0 until N) vals[ii][jj] += av[ii][jj]
    }

    open fun mult(num: Double) { for (ii in 0 until M) for (jj in 0 until N) vals[ii][jj] *= num }

    open fun div(num: Double) = mult(1.0 / num)

    fun mult(aMat: Matrix, bMat: Matrix) {
        if (M != aMat.M || N != bMat.N || aMat.N != bMat.M)
            throw VectorSpaceArgumentException("Wrong dimensions for Matrix.mult(Matrix aMat, Matrix bMat)")
        zero()
        val av = aMat.valuesPtr(); val bv = bMat.valuesPtr()
        for (ii in 0 until M) for (kk in 0 until aMat.N) for (jj in 0 until N)
            vals[ii][jj] += av[ii][kk] * bv[kk][jj]
    }

    fun mult(a: Tuple, b: Tuple) {
        if (M != a.N || M != b.N || M != N)
            throw VectorSpaceArgumentException("Wrong dimensions for Matrix.mult(Tuple a, Tuple b)")
        val av = a.valuesPtr(); val bv = b.valuesPtr()
        for (ii in 0 until M) for (jj in 0 until M) vals[ii][jj] = av[ii] * bv[jj]
    }

    fun normalEqn(a: Matrix) {
        val aT = Matrix(a.N, a.M); aT.transpose(a); mult(aT, a)
    }

    fun normalEqn(a: Matrix, w: Matrix) {
        val aT = Matrix(a.numCols(), a.numRows()); aT.transpose(a)
        val aTw = Matrix(aT.numCols()); aTw.mult(aT, w); mult(aTw, a)
    }

    fun transform(a: Matrix, cov: Matrix) {
        val aT = Matrix(a.numCols(), a.numRows()); aT.transpose(a)
        val aw = Matrix(a.numRows(), a.numCols()); aw.mult(a, cov); mult(aw, aT)
    }

    fun trace(): Double {
        if (!mxm) throw VectorSpaceArgumentException("Can't find trace of non-square matrix $this")
        var ret = 0.0
        for (ii in 0 until M) ret += vals[ii][ii]
        return ret
    }

    fun transpose() {
        if (!mxm) throw VectorSpaceArgumentException("Can't in-place transpose non-square matrix $this")
        for (ii in 0 until M) for (jj in 0 until ii) {
            val tmp = vals[jj][ii]; vals[jj][ii] = vals[ii][jj]; vals[ii][jj] = tmp
        }
    }

    fun transpose(aMat: Matrix) {
        if (M != aMat.N || N != aMat.M)
            throw VectorSpaceArgumentException("Can't transpose - dimensions of input matrix incorrect")
        val av = aMat.valuesPtr()
        for (ii in 0 until M) for (jj in 0 until N) vals[ii][jj] = av[jj][ii]
    }

    internal fun getQR(qMat: Matrix, rMat: Matrix) {
        if (N != rMat.M || N != rMat.N) throw VectorSpaceArgumentException("R needs to be NxN $rMat")
        rMat.zero(); qMat.set(this)
        val qk = Tuple(M); val qj = Tuple(M)
        for (kk in 1..N) {
            qk.setColumn(kk, qMat)
            val rkk = qk.mag()
            if (rkk < EPS) throw SingularMatrixException("This matrix does not appear to be full rank: $this")
            rMat.put(kk, kk, rkk); qk.div(rkk); qMat.setColumn(kk, qk)
            for (jj in (kk + 1)..N) {
                qk.setColumn(kk, qMat); qj.setColumn(jj, qMat)
                rMat.put(kk, jj, qk.dot(qj)); qk.mult(rMat.get(kk, jj))
                qj.minus(qk); qMat.setColumn(jj, qj)
            }
        }
    }

    internal fun solve(y: Tuple, x: Tuple) {
        if (!mxm || M < 1) throw VectorSpaceArgumentException("Can't solve equations with non-square Matrix $this")
        if (y.N != N || x.N != N) throw VectorSpaceArgumentException("Can't solve for x given y (Dimensions wrong)")
        val ro = row_order ?: return
        solve(y.valuesPtr(), x.valuesPtr())
    }

    internal fun solveCH(y: Tuple, x: Tuple) {
        if (!mxm || M < 1) throw VectorSpaceArgumentException("Can't solve equations with non-square Matrix $this")
        if (y.N != N || x.N != N) throw VectorSpaceArgumentException("Can't solve for x given y (Dimensions wrong)")
        val yv = y.valuesPtr(); val xv = x.valuesPtr()
        val bvals = DoubleArray(x.N)
        for (ii in 0 until M) {
            var sum = 0.0
            for (jj in 0 until ii) sum += bvals[jj] * vals[ii][jj]
            bvals[ii] = (yv[ii] - sum) / vals[ii][ii]
        }
        for (ii in M - 1 downTo 0) {
            var sum = 0.0
            for (jj in ii + 1 until M) sum += xv[jj] * vals[jj][ii]
            xv[ii] = (bvals[ii] - sum) / vals[ii][ii]
        }
    }

    fun croutLU() {
        if (!mxm || N < 1) throw VectorSpaceArgumentException("Can't decompose non-square Matrix $this")
        if (order()) throw SingularMatrixException("Can't decompose Matrix: Row of zeros $this")
        decomp()
    }

    fun cholesky() {
        if (!mxm || N < 1) throw VectorSpaceArgumentException("Can't decompose non-square Matrix $this")
        for (kk in 0 until M) {
            for (ii in 0 until kk) {
                if (vals[ii][ii] < EPS) throw SingularMatrixException(
                    "Can't decompose Matrix: Diagonal elements must be positive $this")
                var sum = 0.0
                for (jj in 0 until ii) sum += vals[ii][jj] * vals[kk][jj]
                vals[kk][ii] = (vals[kk][ii] - sum) / vals[ii][ii]
            }
            var sum = 0.0
            for (jj in 0 until kk) sum += vals[kk][jj] * vals[kk][jj]
            val tmp = vals[kk][kk] - sum
            if (tmp < EPS) throw SingularMatrixException(
                "Can't decompose Matrix: Not symmetric positive definite $this")
            vals[kk][kk] = Math.sqrt(tmp)
        }
    }

    fun getRowOrder(ii: Int): Int {
        if (ii <= 0 || ii > M) return 0
        return row_order?.let { it[ii - 1] + 1 } ?: ii
    }

    fun det(): Double {
        if (!mxm || M < 1) throw VectorSpaceArgumentException("Can't find determinant of non-square Matrix $this")
        if (M == 3) return (vals[0][0]*(vals[1][1]*vals[2][2] - vals[2][1]*vals[1][2])
                          - vals[0][1]*(vals[1][0]*vals[2][2] - vals[2][0]*vals[1][2])
                          + vals[0][2]*(vals[1][0]*vals[2][1] - vals[2][0]*vals[1][1]))
        if (M == 2) return (vals[0][0]*vals[1][1] - vals[1][0]*vals[0][1])
        if (M == 1) return vals[0][0]
        if (recursiveMat) {
            if (order()) return 0.0
            val sign = gaussElim()
            return diag_prod() * sign
        }
        val detMatrix = Matrix(this.values(), true)
        return detMatrix.det()
    }

    fun invert() {
        if (!mxm || N < 1) throw VectorSpaceArgumentException("Can't invert non-square Matrix $this")
        if (M < 3) {
            val d = det()
            if (Math.abs(d) < EPS) throw SingularMatrixException("Can't Invert Matrix: Singular, or Near-Singular $this")
            if (M == 1) { vals[0][0] = 1.0 / d } else {
                val tmp = vals[1][1]
                vals[1][1] = vals[0][0] / d; vals[0][0] = tmp / d
                vals[0][1] /= -d; vals[1][0] /= -d
            }
            return
        }
        croutLU()
        if (Math.abs(diag_prod()) < EPS) throw SingularMatrixException("Can't Invert Matrix: Singular, or Near-Singular $this")
        val xmat = Array(N) { DoubleArray(N) }
        val ctup = DoubleArray(N); ctup[0] = 1.0
        solve(ctup, xmat[0])
        for (jj in 1 until N) { ctup[jj - 1] = 0.0; ctup[jj] = 1.0; solve(ctup, xmat[jj]) }
        set(xmat); transpose()
    }

    private fun solve(cvals: DoubleArray, xvals: DoubleArray) {
        val ro = row_order!!
        xvals[0] = cvals[ro[0]] / vals[ro[0]][0]
        for (ii in 1 until M) {
            var sum = 0.0
            for (jj in 0 until ii) sum += vals[ro[ii]][jj] * xvals[jj]
            xvals[ii] = (cvals[ro[ii]] - sum) / vals[ro[ii]][ii]
        }
        for (ii in N - 2 downTo 0) {
            var sum = 0.0
            for (jj in ii + 1 until N) sum += vals[ro[ii]][jj] * xvals[jj]
            xvals[ii] -= sum
        }
    }

    private fun decomp() {
        var jj = 0; pivot(jj)
        for (jj2 in 1 until N) vals[row_order!![0]][jj2] /= vals[row_order!![0]][0]
        for (jj3 in 1 until N - 1) {
            for (ii in jj3 until M) {
                var sum = 0.0
                for (kk in 0 until jj3) sum += vals[row_order!![ii]][kk] * vals[row_order!![kk]][jj3]
                vals[row_order!![ii]][jj3] -= sum
            }
            pivot(jj3)
            for (kk in jj3 + 1 until M) {
                var sum = 0.0
                for (ii in 0 until jj3) sum += vals[row_order!![jj3]][ii] * vals[row_order!![ii]][kk]
                vals[row_order!![jj3]][kk] = (vals[row_order!![jj3]][kk] - sum) / vals[row_order!![jj3]][jj3]
            }
        }
        var sum = 0.0
        for (kk in 0 until N - 1) sum += vals[row_order!![M - 1]][kk] * vals[row_order!![kk]][N - 1]
        vals[row_order!![M - 1]][N - 1] -= sum
    }

    private fun order(): Boolean {
        if (row_order == null) { row_order = IntArray(M); row_scale = DoubleArray(M) }
        val ro = row_order!!; val rs = row_scale!!
        var singular = false
        for (ii in 0 until M) {
            ro[ii] = ii; rs[ii] = Math.abs(vals[ii][0])
            for (jj in 1 until N) { val s = Math.abs(vals[ii][jj]); if (s > rs[ii]) rs[ii] = s }
            if (rs[ii] < EPS) singular = true
        }
        return singular
    }

    private fun pivot(jj: Int) {
        val ro = row_order!!; val rs = row_scale!!
        var pivit = jj; var big = Math.abs(vals[ro[jj]][jj] / rs[ro[jj]])
        for (ii in jj + 1 until M) {
            val dum = Math.abs(vals[ro[ii]][jj] / rs[ro[ii]])
            if (dum > big) { big = dum; pivit = ii }
        }
        val idum = ro[pivit]; ro[pivit] = ro[jj]; ro[jj] = idum
    }

    private fun gaussElim(): Double {
        val ro = row_order!!
        for (kk in 0 until N - 1) {
            pivot(kk)
            for (ii in kk + 1 until M) {
                val factor = vals[ro[ii]][kk] / vals[ro[kk]][kk]
                for (jj in kk + 1 until N) vals[ro[ii]][jj] -= factor * vals[ro[kk]][jj]
            }
        }
        var swaps = 0
        for (ii in 0 until M) if (ii != ro[ii]) swaps++
        if (swaps == 0) return 1.0
        swaps--
        return if (swaps % 2 != 0) -1.0 else 1.0
    }

    private fun diag_prod(): Double {
        val ro = row_order
        var prod = 1.0
        if (ro == null) for (ii in 0 until M) prod *= vals[ii][ii]
        else for (ii in 0 until M) prod *= vals[ro[ii]][ii]
        return prod
    }

    override fun toString(): String = buildString {
        for (ii in 0 until M) {
            for (jj in 0 until N) append("  ${vals[ii][jj]}")
            append('\n')
        }
    }
}
