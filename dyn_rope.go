package b2

import (
	"math"
)

var StretchingModel = struct {
	PbdStretchingModel  uint8
	XpbdStretchingModel uint8
}{
	PbdStretchingModel:  1,
	XpbdStretchingModel: 2,
}

var BendingModel = struct {
	SpringAngleBendingModel uint8
	PbdAngleBendingModel    uint8
	XpbdAngleBendingModel   uint8
	PbdDistanceBendingModel uint8
	PbdHeightBendingModel   uint8
	PbdTriangleBendingModel uint8
}{
	SpringAngleBendingModel: 1,
	PbdAngleBendingModel:    2,
	XpbdAngleBendingModel:   3,
	PbdDistanceBendingModel: 4,
	PbdHeightBendingModel:   5,
	PbdTriangleBendingModel: 6,
}

type RopeTuning struct {
	StretchingModel    uint8
	BendingModel       uint8
	Damping            float64
	StretchStiffness   float64
	StretchHertz       float64
	StretchDamping     float64
	BendStiffness      float64
	BendHertz          float64
	BendDamping        float64
	Isometric          bool
	FixedEffectiveMass bool
	WarmStart          bool
}

func MakeRopeTuning() RopeTuning {
	res := RopeTuning{}
	res.StretchingModel = StretchingModel.PbdStretchingModel
	res.BendingModel = BendingModel.PbdAngleBendingModel
	res.StretchStiffness = 1.0
	res.StretchHertz = 1.0
	res.BendStiffness = 0.5
	res.BendHertz = 1.0
	return res
}

type RopeDef struct {
	Position Vec2
	Vertices []Vec2
	Count    int
	Masses   []float64
	Gravity  Vec2
	Tuning   RopeTuning
}

func MakeRopeDef() RopeDef {
	res := RopeDef{}
	res.Tuning = MakeRopeTuning()
	return res
}

type RopeStretch struct {
	I1       int
	I2       int
	InvMass1 float64
	InvMass2 float64
	L        float64
	Lambda   float64
	Spring   float64
	Damper   float64
}

type RopeBend struct {
	I1               int
	I2               int
	I3               int
	InvMass1         float64
	InvMass2         float64
	InvMass3         float64
	InvEffectiveMass float64
	Lambda           float64
	L1               float64
	L2               float64
	alpha1           float64
	alpha2           float64
	Spring           float64
	Damper           float64
}

type Rope struct {
	M_position Vec2

	M_count        int
	M_stretchCount int
	M_bendCount    int

	M_stretchConstraints []RopeStretch
	M_bendConstraints    []RopeBend

	M_bindPositions []Vec2
	M_ps            []Vec2
	M_p0s           []Vec2
	M_vs            []Vec2

	M_invMasses []float64
	M_gravity   Vec2

	M_tuning RopeTuning
}

func (rope Rope) GetVertexCount() int {
	return rope.M_count
}

func (rope Rope) GetVertices() []Vec2 {
	return rope.M_ps
}

func MakeRope() Rope {
	res := Rope{}
	res.M_tuning = MakeRopeTuning()
	return res
}

func (rope *Rope) Destroy() {
	rope.M_stretchConstraints = nil
	rope.M_bendConstraints = nil
	rope.M_bindPositions = nil
	rope.M_ps = nil
	rope.M_p0s = nil
	rope.M_vs = nil
	rope.M_invMasses = nil
}

func (rope *Rope) Create(def *RopeDef) {
	assert(def.Count >= 3)
	rope.M_position = def.Position
	rope.M_count = def.Count
	rope.M_bindPositions = make([]Vec2, rope.M_count)
	rope.M_ps = make([]Vec2, rope.M_count)
	rope.M_p0s = make([]Vec2, rope.M_count)
	rope.M_vs = make([]Vec2, rope.M_count)
	rope.M_invMasses = make([]float64, rope.M_count)

	for i := 0; i < rope.M_count; i++ {
		rope.M_bindPositions[i] = def.Vertices[i]
		rope.M_ps[i] = Vec2Add(def.Vertices[i], rope.M_position)
		rope.M_p0s[i] = Vec2Add(def.Vertices[i], rope.M_position)
		rope.M_vs[i].SetZero()

		m := def.Masses[i]
		if m > 0.0 {
			rope.M_invMasses[i] = 1.0 / m
		} else {
			rope.M_invMasses[i] = 0.0
		}
	}

	rope.M_stretchCount = rope.M_count - 1
	rope.M_bendCount = rope.M_count - 2

	rope.M_stretchConstraints = make([]RopeStretch, rope.M_stretchCount)
	rope.M_bendConstraints = make([]RopeBend, rope.M_bendCount)

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]

		c.I1 = i
		c.I2 = i + 1
		c.L = Vec2Distance(p1, p2)
		c.InvMass1 = rope.M_invMasses[i]
		c.InvMass2 = rope.M_invMasses[i+1]
		c.Lambda = 0.0
		c.Damper = 0.0
		c.Spring = 0.0
	}

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[i]
		p2 := rope.M_ps[i+1]
		p3 := rope.M_ps[i+2]

		c.I1 = i
		c.I2 = i + 1
		c.I3 = i + 2
		c.InvMass1 = rope.M_invMasses[i]
		c.InvMass2 = rope.M_invMasses[i+1]
		rope.M_bendConstraints[i].InvMass3 = rope.M_invMasses[i+2]
		rope.M_bendConstraints[i].InvEffectiveMass = 0.0
		rope.M_bendConstraints[i].L1 = Vec2Distance(p1, p2)
		rope.M_bendConstraints[i].L2 = Vec2Distance(p2, p3)
		rope.M_bendConstraints[i].Lambda = 0.0

		// Pre-compute effective mass (TODO use flattened config)
		e1 := Vec2Sub(p2, p1)
		e2 := Vec2Sub(p3, p2)
		L1sqr := e1.LengthSquared()
		L2sqr := e2.LengthSquared()

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		Jd1 := Vec2MulScalar((-1.0 / L1sqr), e1.Skew())
		Jd2 := Vec2MulScalar((1.0 / L2sqr), e2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		c.InvEffectiveMass = c.InvMass1*Vec2Dot(J1, J1) + c.InvMass2*Vec2Dot(J2, J2) + c.InvMass3*Vec2Dot(J3, J3)

		r := Vec2Sub(p3, p1)

		rr := r.LengthSquared()
		if rr == 0.0 {
			continue
		}

		// a1 = h2 / (h1 + h2)
		// a2 = h1 / (h1 + h2)
		c.alpha1 = Vec2Dot(e2, r) / rr
		c.alpha2 = Vec2Dot(e1, r) / rr
	}

	rope.M_gravity = def.Gravity

	rope.SetTuning(def.Tuning)
}

func (rope *Rope) SetTuning(tuning RopeTuning) {
	rope.M_tuning = tuning

	// Pre-compute spring and damper values based on tuning

	bendOmega := 2.0 * pi * rope.M_tuning.BendHertz

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		L1sqr := c.L1 * c.L1
		L2sqr := c.L2 * c.L2

		if L1sqr*L2sqr == 0.0 {
			c.Spring = 0.0
			c.Damper = 0.0
			continue
		}

		// Flatten the triangle formed by the two edges
		J2 := 1.0/c.L1 + 1.0/c.L2
		sum := c.InvMass1/L1sqr + c.InvMass2*J2*J2 + c.InvMass3/L2sqr
		if sum == 0.0 {
			c.Spring = 0.0
			c.Damper = 0.0
			continue
		}

		mass := 1.0 / sum

		c.Spring = mass * bendOmega * bendOmega
		c.Damper = 2.0 * mass * rope.M_tuning.BendDamping * bendOmega
	}

	stretchOmega := 2.0 * pi * rope.M_tuning.StretchHertz

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		sum := c.InvMass1 + c.InvMass2
		if sum == 0.0 {
			continue
		}

		mass := 1.0 / sum

		c.Spring = mass * stretchOmega * stretchOmega
		c.Damper = 2.0 * mass * rope.M_tuning.StretchDamping * stretchOmega
	}
}

func (rope *Rope) Step(dt float64, iterations int, position Vec2) {
	if dt == 0.0 {
		return
	}

	inv_dt := 1.0 / dt
	d := math.Exp(-dt * rope.M_tuning.Damping)

	// Apply gravity and damping
	for i := 0; i < rope.M_count; i++ {
		if rope.M_invMasses[i] > 0.0 {
			rope.M_vs[i].OperatorScalarMulInplace(d)
			rope.M_vs[i].OperatorPlusInplace(Vec2MulScalar(dt, rope.M_gravity))
		} else {
			rope.M_vs[i] = Vec2MulScalar(inv_dt, Vec2Sub(Vec2Add(rope.M_bindPositions[i], position), rope.M_p0s[i]))
		}
	}

	// Apply bending spring
	if rope.M_tuning.BendingModel == BendingModel.SpringAngleBendingModel {
		rope.ApplyBendForces(dt)
	}

	for i := 0; i < rope.M_bendCount; i++ {
		rope.M_bendConstraints[i].Lambda = 0.0
	}

	for i := 0; i < rope.M_stretchCount; i++ {
		rope.M_stretchConstraints[i].Lambda = 0.0
	}

	// Update position
	for i := 0; i < rope.M_count; i++ {
		rope.M_ps[i].OperatorPlusInplace(Vec2MulScalar(dt, rope.M_vs[i]))
	}

	// Solve constraints
	for range iterations {
		switch rope.M_tuning.BendingModel {
		case BendingModel.PbdAngleBendingModel:
			rope.SolveBend_PBD_Angle()
		case BendingModel.XpbdAngleBendingModel:
			rope.SolveBend_XPBD_Angle(dt)
		case BendingModel.PbdDistanceBendingModel:
			rope.SolveBend_PBD_Distance()
		case BendingModel.PbdHeightBendingModel:
			rope.SolveBend_PBD_Height()
		case BendingModel.PbdTriangleBendingModel:
			rope.SolveBend_PBD_Triangle()
		}

		switch rope.M_tuning.StretchingModel {
		case StretchingModel.PbdStretchingModel:
			rope.SolveStretch_PBD()
		case StretchingModel.XpbdStretchingModel:
			rope.SolveStretch_XPBD(dt)
		}
	}

	// Constrain velocity
	for i := 0; i < rope.M_count; i++ {
		rope.M_vs[i] = Vec2MulScalar(inv_dt, Vec2Sub(rope.M_ps[i], rope.M_p0s[i]))
		rope.M_p0s[i] = rope.M_ps[i]
	}
}

func (rope *Rope) Reset(position Vec2) {
	rope.M_position = position

	for i := 0; i < rope.M_count; i++ {
		rope.M_ps[i] = Vec2Add(rope.M_bindPositions[i], rope.M_position)
		rope.M_p0s[i] = Vec2Add(rope.M_bindPositions[i], rope.M_position)
		rope.M_vs[i].SetZero()
	}

	for i := 0; i < rope.M_bendCount; i++ {
		rope.M_bendConstraints[i].Lambda = 0.0
	}

	for i := 0; i < rope.M_stretchCount; i++ {
		rope.M_stretchConstraints[i].Lambda = 0.0
	}
}

func (rope *Rope) SolveStretch_PBD() {
	stiffness := rope.M_tuning.StretchStiffness

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]

		d := Vec2Sub(p2, p1)
		L := d.Normalize()

		sum := c.InvMass1 + c.InvMass2
		if sum == 0.0 {
			continue
		}

		s1 := c.InvMass1 / sum
		s2 := c.InvMass2 / sum

		p1.OperatorMinusInplace(Vec2MulScalar(stiffness*s1*(c.L-L), d))
		p2.OperatorPlusInplace(Vec2MulScalar(stiffness*s2*(c.L-L), d))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
	}
}

func (rope *Rope) SolveStretch_XPBD(dt float64) {
	assert(dt > 0.0)

	for i := 0; i < rope.M_stretchCount; i++ {
		c := &rope.M_stretchConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]

		dp1 := Vec2Sub(p1, rope.M_p0s[c.I1])
		dp2 := Vec2Sub(p2, rope.M_p0s[c.I2])

		u := Vec2Sub(p2, p1)
		L := u.Normalize()

		J1 := u.OperatorNegate()
		J2 := u

		sum := c.InvMass1 + c.InvMass2
		if sum == 0.0 {
			continue
		}

		alpha := 1.0 / (c.Spring * dt * dt) // 1 / kg
		beta := dt * dt * c.Damper          // kg * s
		sigma := alpha * beta / dt          // non-dimensional
		C := L - c.L

		// This is using the initial velocities
		Cdot := Vec2Dot(J1, dp1) + Vec2Dot(J2, dp2)

		B := C + alpha*c.Lambda + sigma*Cdot
		sum2 := (1.0+sigma)*sum + alpha

		impulse := -B / sum2

		p1.OperatorPlusInplace(Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(Vec2MulScalar((c.InvMass2 * impulse), J2))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		c.Lambda += impulse
	}
}

func (rope *Rope) SolveBend_PBD_Angle() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		d1 := Vec2Sub(p2, p1)
		d2 := Vec2Sub(p3, p2)
		a := Vec2Cross(d1, d2)
		b := Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		var L1sqr float64
		var L2sqr float64

		if rope.M_tuning.Isometric {
			L1sqr = c.L1 * c.L1
			L2sqr = c.L2 * c.L2
		} else {
			L1sqr = d1.LengthSquared()
			L2sqr = d2.LengthSquared()
		}

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		Jd1 := Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		var sum float64
		if rope.M_tuning.FixedEffectiveMass {
			sum = c.InvEffectiveMass
		} else {
			sum = c.InvMass1*Vec2Dot(J1, J1) + c.InvMass2*Vec2Dot(J2, J2) + c.InvMass3*Vec2Dot(J3, J3)
		}

		if sum == 0.0 {
			sum = c.InvEffectiveMass
		}

		impulse := -stiffness * angle / sum

		p1.OperatorPlusInplace(Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(Vec2MulScalar((c.InvMass2 * impulse), J2))
		p3.OperatorPlusInplace(Vec2MulScalar((c.InvMass3 * impulse), J3))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		rope.M_ps[c.I3] = p3
	}
}

func (rope *Rope) SolveBend_XPBD_Angle(dt float64) {
	assert(dt > 0.0)

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		dp1 := Vec2Sub(p1, rope.M_p0s[c.I1])
		dp2 := Vec2Sub(p2, rope.M_p0s[c.I2])
		dp3 := Vec2Sub(p3, rope.M_p0s[c.I3])

		d1 := Vec2Sub(p2, p1)
		d2 := Vec2Sub(p3, p2)

		var L1sqr float64
		var L2sqr float64

		if rope.M_tuning.Isometric {
			L1sqr = c.L1 * c.L1
			L2sqr = c.L2 * c.L2
		} else {
			L1sqr = d1.LengthSquared()
			L2sqr = d2.LengthSquared()
		}

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		a := Vec2Cross(d1, d2)
		b := Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		Jd1 := Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		var sum float64
		if rope.M_tuning.FixedEffectiveMass {
			sum = c.InvEffectiveMass
		} else {
			sum = c.InvMass1*Vec2Dot(J1, J1) + c.InvMass2*Vec2Dot(J2, J2) + c.InvMass3*Vec2Dot(J3, J3)
		}

		if sum == 0.0 {
			continue
		}

		alpha := 1.0 / (c.Spring * dt * dt)
		beta := dt * dt * c.Damper
		sigma := alpha * beta / dt
		C := angle

		// This is using the initial velocities
		Cdot := Vec2Dot(J1, dp1) + Vec2Dot(J2, dp2) + Vec2Dot(J3, dp3)

		B := C + alpha*c.Lambda + sigma*Cdot
		sum2 := (1.0+sigma)*sum + alpha

		impulse := -B / sum2

		p1.OperatorPlusInplace(Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(Vec2MulScalar((c.InvMass2 * impulse), J2))
		p3.OperatorPlusInplace(Vec2MulScalar((c.InvMass3 * impulse), J3))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		rope.M_ps[c.I3] = p3
		c.Lambda += impulse
	}
}

func (rope *Rope) ApplyBendForces(dt float64) {
	// omega = 2 * pi * hz
	omega := 2.0 * pi * rope.M_tuning.BendHertz

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		v1 := rope.M_vs[c.I1]
		v2 := rope.M_vs[c.I2]
		v3 := rope.M_vs[c.I3]

		d1 := Vec2Sub(p2, p1)
		d2 := Vec2Sub(p3, p2)

		var L1sqr float64
		var L2sqr float64

		if rope.M_tuning.Isometric {
			L1sqr = c.L1 * c.L1
			L2sqr = c.L2 * c.L2
		} else {
			L1sqr = d1.LengthSquared()
			L2sqr = d2.LengthSquared()
		}

		if L1sqr*L2sqr == 0.0 {
			continue
		}

		a := Vec2Cross(d1, d2)
		b := Vec2Dot(d1, d2)

		angle := math.Atan2(a, b)

		Jd1 := Vec2MulScalar((-1.0 / L1sqr), d1.Skew())
		Jd2 := Vec2MulScalar((1.0 / L2sqr), d2.Skew())

		J1 := Jd1.OperatorNegate()
		J2 := Vec2Sub(Jd1, Jd2)
		J3 := Jd2

		var sum float64
		if rope.M_tuning.FixedEffectiveMass {
			sum = c.InvEffectiveMass
		} else {
			sum = c.InvMass1*Vec2Dot(J1, J1) + c.InvMass2*Vec2Dot(J2, J2) + c.InvMass3*Vec2Dot(J3, J3)
		}

		if sum == 0.0 {
			continue
		}

		mass := 1.0 / sum

		spring := mass * omega * omega
		damper := 2.0 * mass * rope.M_tuning.BendDamping * omega

		C := angle
		Cdot := Vec2Dot(J1, v1) + Vec2Dot(J2, v2) + Vec2Dot(J3, v3)

		impulse := -dt * (spring*C + damper*Cdot)

		rope.M_vs[c.I1].OperatorPlusInplace(Vec2MulScalar((c.InvMass1 * impulse), J1))
		rope.M_vs[c.I2].OperatorPlusInplace(Vec2MulScalar((c.InvMass2 * impulse), J2))
		rope.M_vs[c.I3].OperatorPlusInplace(Vec2MulScalar((c.InvMass3 * impulse), J3))
	}
}

func (rope *Rope) SolveBend_PBD_Distance() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {

		c := &rope.M_bendConstraints[i]

		i1 := c.I1
		i2 := c.I3

		p1 := rope.M_ps[i1]
		p2 := rope.M_ps[i2]

		d := Vec2Sub(p2, p1)
		L := d.Normalize()

		sum := c.InvMass1 + c.InvMass3
		if sum == 0.0 {
			continue
		}

		s1 := c.InvMass1 / sum
		s2 := c.InvMass3 / sum

		p1.OperatorMinusInplace(Vec2MulScalar(stiffness*s1*(c.L1+c.L2-L), d))
		p2.OperatorPlusInplace(Vec2MulScalar(stiffness*s2*(c.L1+c.L2-L), d))

		rope.M_ps[i1] = p1
		rope.M_ps[i2] = p2
	}
}

// Constraint based implementation of:
// P. Volino: Simple Linear Bending Stiffness in Particle Systems
func (rope *Rope) SolveBend_PBD_Height() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		p1 := rope.M_ps[c.I1]
		p2 := rope.M_ps[c.I2]
		p3 := rope.M_ps[c.I3]

		// Barycentric coordinates are held constant
		d := Vec2Sub(Vec2Add(Vec2MulScalar(c.alpha1, p1), Vec2MulScalar(c.alpha2, p3)), p2)
		dLen := d.Length()

		if dLen == 0.0 {
			continue
		}

		dHat := Vec2MulScalar((1.0 / dLen), d)

		J1 := Vec2MulScalar(c.alpha1, dHat)
		J2 := dHat.OperatorNegate()
		J3 := Vec2MulScalar(c.alpha2, dHat)

		sum := c.InvMass1*c.alpha1*c.alpha1 + c.InvMass2 + c.InvMass3*c.alpha2*c.alpha2

		if sum == 0.0 {
			continue
		}

		C := dLen
		mass := 1.0 / sum
		impulse := -stiffness * mass * C

		p1.OperatorPlusInplace(Vec2MulScalar((c.InvMass1 * impulse), J1))
		p2.OperatorPlusInplace(Vec2MulScalar((c.InvMass2 * impulse), J2))
		p3.OperatorPlusInplace(Vec2MulScalar((c.InvMass3 * impulse), J3))

		rope.M_ps[c.I1] = p1
		rope.M_ps[c.I2] = p2
		rope.M_ps[c.I3] = p3
	}
}

// M. Kelager: A Triangle Bending Constraint Model for PBD
func (rope *Rope) SolveBend_PBD_Triangle() {
	stiffness := rope.M_tuning.BendStiffness

	for i := 0; i < rope.M_bendCount; i++ {
		c := &rope.M_bendConstraints[i]

		b0 := rope.M_ps[c.I1]
		v := rope.M_ps[c.I2]
		b1 := rope.M_ps[c.I3]

		wb0 := c.InvMass1
		wv := c.InvMass2
		wb1 := c.InvMass3

		W := wb0 + wb1 + 2.0*wv
		invW := stiffness / W

		d := Vec2Sub(v, Vec2MulScalar(1.0/3.0, Vec2Add(Vec2Add(b0, v), b1)))

		db0 := Vec2MulScalar(2.0*wb0*invW, d)
		dv := Vec2MulScalar(-4.0*wv*invW, d)
		db1 := Vec2MulScalar(2.0*wb1*invW, d)

		b0.OperatorPlusInplace(db0)
		v.OperatorPlusInplace(dv)
		b1.OperatorPlusInplace(db1)

		rope.M_ps[c.I1] = b0
		rope.M_ps[c.I2] = v
		rope.M_ps[c.I3] = b1
	}
}

//void b2Rope::Draw(b2Draw* draw) const
//{
//	b2Color c(0.4f, 0.5f, 0.7f);
//	b2Color pg(0.1f, 0.8f, 0.1f);
//	b2Color pd(0.7f, 0.2f, 0.4f);
//
//	for (int32 i = 0; i < m_count - 1; ++i)
//	{
//		draw->DrawSegment(m_ps[i], m_ps[i+1], c);
//
//		const b2Color& pc = m_invMasses[i] > 0.0f ? pd : pg;
//		draw->DrawPoint(m_ps[i], 5.0f, pc);
//	}
//
//	const b2Color& pc = m_invMasses[m_count - 1] > 0.0f ? pd : pg;
//	draw->DrawPoint(m_ps[m_count - 1], 5.0f, pc);
//}
