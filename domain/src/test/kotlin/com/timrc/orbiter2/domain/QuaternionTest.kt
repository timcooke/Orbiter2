package com.timrc.orbiter2.domain

import com.timrc.orbiter2.domain.enums.Basis3D
import com.timrc.orbiter2.domain.math.Matrix3X3
import com.timrc.orbiter2.domain.math.Quaternion
import com.timrc.orbiter2.domain.math.Tuple3D
import org.junit.Assert.assertEquals
import org.junit.Test

/**
 * Tests for Quaternion / Matrix3X3 consistency.
 *
 * VSE convention: rotX/rotY/rotZ produce *frame-rotation* (passive) DCMs.
 * Quaternion.set(dcm) extracts q such that:   fRot(q, v) = dcm * v
 *                                       and:   vRot(q, v) = dcm^T * v
 *
 * Four-pass loop mirrors the structure of TestQuaternion.java from the cognition repo.
 */
class QuaternionTest {

    private val TOL = 1.0e-12

    @Test
    fun quaternionMatchesDcm() {
        var xrot = Math.toRadians( 30.0)
        var yrot = Math.toRadians(-95.0)
        var zrot = Math.toRadians(195.0)

        val rot3 = Matrix3X3(); val rot2 = Matrix3X3(); val rot1 = Matrix3X3()

        for (ii in 0..3) {
            when (ii) {
                1 -> { xrot *= -1.0; yrot *= -1.0; zrot *= -1.0 }
                2 -> { xrot *= -1.5; yrot *= -1.5; zrot *= -1.5 }
                3 -> { xrot -= 3.0;  yrot -= 3.0;  zrot -= 3.0 }
            }

            rot3.rotX(xrot); rot2.rotY(yrot); rot1.rotZ(zrot)

            val rot21 = Matrix3X3(); rot21.mult(rot2, rot1)
            val rot321 = Matrix3X3(); rot321.mult(rot3, rot21)

            // Extract quaternion from DCM: fRot(q, v) == rot321 * v
            val q = Quaternion(); q.set(rot321)

            val v0 = Tuple3D(3.0, 2.0, 1.0)

            // DCM rotation: vm = rot321 * v0
            val vm = Tuple3D(); vm.mult(rot321, v0)

            // Quaternion frame-rotation: vq = fRot(q, v0) must equal vm
            val vq = Tuple3D(); vq.fRot(q, v0)

            val dvx = vq.get(Basis3D.I) - vm.get(Basis3D.I)
            val dvy = vq.get(Basis3D.J) - vm.get(Basis3D.J)
            val dvz = vq.get(Basis3D.K) - vm.get(Basis3D.K)
            val err1 = Math.sqrt(dvx*dvx + dvy*dvy + dvz*dvz)
            assertEquals("Matrix3X3 vs Quaternion (pass $ii)", 0.0, err1, TOL)

            // Round-trip: vRot(q, fRot(q, v0)) should recover v0
            val v02 = Tuple3D(); v02.vRot(q, vq)
            val dx = v0.get(Basis3D.I) - v02.get(Basis3D.I)
            val dy = v0.get(Basis3D.J) - v02.get(Basis3D.J)
            val dz = v0.get(Basis3D.K) - v02.get(Basis3D.K)
            val err2 = Math.sqrt(dx*dx + dy*dy + dz*dz)
            assertEquals("Quaternion fRot/vRot round-trip (pass $ii)", 0.0, err2, TOL)

            // Composed quaternions: q1 q2 q3 should match single DCM (fRot convention)
            val q1 = Quaternion(); q1.set(rot1)
            val q2 = Quaternion(); q2.set(rot2)
            val q3 = Quaternion(); q3.set(rot3)
            val q12 = Quaternion(); q12.mult(q1, q2)
            val q123 = Quaternion(); q123.mult(q12, q3)
            val vqmult = Tuple3D(); vqmult.fRot(q123, v0)
            val dx3 = vm.get(Basis3D.I) - vqmult.get(Basis3D.I)
            val dy3 = vm.get(Basis3D.J) - vqmult.get(Basis3D.J)
            val dz3 = vm.get(Basis3D.K) - vqmult.get(Basis3D.K)
            val err3 = Math.sqrt(dx3*dx3 + dy3*dy3 + dz3*dz3)
            assertEquals("Quaternion multiply (pass $ii)", 0.0, err3, TOL)
        }
    }
}
