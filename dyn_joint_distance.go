package b2

import (
	"fmt"
	"math"
)

// Distance joint definition. This requires defining an anchor point on both
// bodies and the non-zero distance of the distance joint. The definition uses
// local anchor points so that the initial configuration can violate the
// constraint slightly. This helps when saving and loading a game.
type DistanceJointDef struct {
	JointDef

	// The local anchor point relative to bodyA's origin.
	LocalAnchorA Vec2

	// The local anchor point relative to bodyB's origin.
	LocalAnchorB Vec2

	// The rest length of this joint. Clamped to a stable minimum value.
	Length float64

	// Minimum length. Clamped to a stable minimum value.
	MinLength float64

	// Maximum length. Must be greater than or equal to the minimum length.
	MaxLength float64

	// The linear stiffness in N/m.
	Stiffness float64

	// The linear damping in N*s/m.
	Damping float64
}

func MakeDistanceJointDef() DistanceJointDef {
	res := DistanceJointDef{
		JointDef: DefaultJointDef(),
	}

	res.Type = DistanceJointType
	res.LocalAnchorA.Set(0.0, 0.0)
	res.LocalAnchorB.Set(0.0, 0.0)
	res.Length = 1.0
	res.MinLength = 0.0
	res.MaxLength = maxFloat
	res.Stiffness = 0.0
	res.Damping = 0.0

	return res
}

// A distance joint constrains two points on two bodies to remain at a fixed
// distance from each other. You can view this as a massless, rigid rod.
type DistanceJoint struct {
	*Joint

	M_stiffness float64
	M_damping   float64
	M_bias      float64
	M_length    float64
	M_minLength float64
	M_maxLength float64

	// Solver shared
	M_localAnchorA Vec2
	M_localAnchorB Vec2
	M_gamma        float64
	M_impulse      float64
	M_lowerImpulse float64
	M_upperImpulse float64

	// Solver temp
	M_indexA        int
	M_indexB        int
	M_u             Vec2
	M_rA            Vec2
	M_rB            Vec2
	M_localCenterA  Vec2
	M_localCenterB  Vec2
	M_currentLength float64
	M_invMassA      float64
	M_invMassB      float64
	M_invIA         float64
	M_invIB         float64
	M_softMass      float64
	M_mass          float64
}

// The local anchor point relative to bodyA's origin.
func (joint DistanceJoint) GetLocalAnchorA() Vec2 {
	return joint.M_localAnchorA
}

// The local anchor point relative to bodyB's origin.
func (joint DistanceJoint) GetLocalAnchorB() Vec2 {
	return joint.M_localAnchorB
}

// Get the rest length
func (joint DistanceJoint) GetLength() float64 {
	return joint.M_length
}

// Set the rest length
// @returns clamped rest length
func (joint *DistanceJoint) SetLength(length float64) float64 {
	joint.M_impulse = 0.0
	joint.M_length = math.Max(linearSlop, length)
	return joint.M_length
}

// Get the minimum length
func (joint DistanceJoint) GetMinLength() float64 {
	return joint.M_minLength
}

// Set the minimum length
// @returns the clamped minimum length
func (joint *DistanceJoint) SetMinLength(minLength float64) float64 {
	joint.M_lowerImpulse = 0.0
	joint.M_minLength = FloatClamp(minLength, linearSlop, joint.M_maxLength)
	return joint.M_minLength
}

// Get the maximum length
func (joint DistanceJoint) GetMaxLength() float64 {
	return joint.M_maxLength
}

// Set the maximum length
// @returns the clamped maximum length
func (joint *DistanceJoint) SetMaxLength(maxLength float64) float64 {
	joint.M_upperImpulse = 0.0
	joint.M_maxLength = math.Max(maxLength, joint.M_minLength)
	return joint.M_maxLength
}

// Get the current length
func (joint DistanceJoint) GetCurrentLength() float64 {
	pA := joint.bodyA.WorldPoint(joint.M_localAnchorA)
	pB := joint.bodyB.WorldPoint(joint.M_localAnchorB)
	d := Vec2Sub(pB, pA)
	length := d.Length()
	return length
}

// Set the linear stiffness in N/m
func (joint *DistanceJoint) SetStiffness(stiffness float64) {
	joint.M_stiffness = stiffness
}

// Get the linear stiffness in N/m
func (joint DistanceJoint) GetStiffness() float64 {
	return joint.M_stiffness
}

// Set linear damping in N*s/m
func (joint *DistanceJoint) SetDamping(damping float64) {
	joint.M_damping = damping
}

// Get linear damping in N*s/m
func (joint DistanceJoint) GetDamping() float64 {
	return joint.M_damping
}

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k *

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

// Initialize the bodies, anchors, and rest length using world space anchors.
// The minimum and maximum lengths are set to the rest length.
func (joint *DistanceJointDef) Initialize(b1 *Body, b2 *Body, anchor1 Vec2, anchor2 Vec2) {
	joint.BodyA = b1
	joint.BodyB = b2
	joint.LocalAnchorA = joint.BodyA.LocalPoint(anchor1)
	joint.LocalAnchorB = joint.BodyB.LocalPoint(anchor2)
	d := Vec2Sub(anchor2, anchor1)
	joint.Length = math.Max(d.Length(), linearSlop)
	joint.MinLength = joint.Length
	joint.MaxLength = joint.Length
}

func MakeDistanceJoint(def *DistanceJointDef) *DistanceJoint {
	res := DistanceJoint{
		Joint: MakeJoint(def),
	}

	res.M_localAnchorA = def.LocalAnchorA
	res.M_localAnchorB = def.LocalAnchorB
	res.M_length = math.Max(def.Length, linearSlop)
	res.M_minLength = math.Max(def.MinLength, linearSlop)
	res.M_maxLength = math.Max(def.MaxLength, res.M_minLength)
	res.M_stiffness = def.Stiffness
	res.M_damping = def.Damping
	res.M_gamma = 0.0
	res.M_bias = 0.0
	res.M_impulse = 0.0
	res.M_lowerImpulse = 0.0
	res.M_upperImpulse = 0.0
	res.M_currentLength = 0.0

	return &res
}

func (joint *DistanceJoint) InitVelocityConstraints(data SolverData) {
	joint.M_indexA = joint.bodyA.islandIndex
	joint.M_indexB = joint.bodyB.islandIndex
	joint.M_localCenterA = joint.bodyA.sweep.LocalCenter
	joint.M_localCenterB = joint.bodyB.sweep.LocalCenter
	joint.M_invMassA = joint.bodyA.invMass
	joint.M_invMassB = joint.bodyB.invMass
	joint.M_invIA = joint.bodyA.invInertia
	joint.M_invIB = joint.bodyB.invInertia

	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W

	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	joint.M_rA = RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	joint.M_rB = RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	joint.M_u = Vec2Sub(Vec2Sub(Vec2Add(cB, joint.M_rB), cA), joint.M_rA)

	// Handle singularity.
	joint.M_currentLength = joint.M_u.Length()
	if joint.M_currentLength > linearSlop {
		joint.M_u.OperatorScalarMulInplace(1.0 / joint.M_currentLength)
	} else {
		joint.M_u.Set(0.0, 0.0)
		joint.M_mass = 0.0
		joint.M_impulse = 0.0
		joint.M_lowerImpulse = 0.0
		joint.M_upperImpulse = 0.0
	}

	crAu := Vec2Cross(joint.M_rA, joint.M_u)
	crBu := Vec2Cross(joint.M_rB, joint.M_u)
	invMass := joint.M_invMassA + joint.M_invIA*crAu*crAu + joint.M_invMassB + joint.M_invIB*crBu*crBu
	if invMass != 0.0 {
		joint.M_mass = 1.0 / invMass
	} else {
		joint.M_mass = 0.0
	}

	if joint.M_stiffness > 0.0 && joint.M_minLength < joint.M_maxLength {
		// soft
		C := joint.M_currentLength - joint.M_length

		d := joint.M_damping
		k := joint.M_stiffness

		// magic formulas
		h := data.Step.Dt

		// gamma = 1 / (h * (d + h * k))
		// the extra factor of h in the denominator is since the lambda is an impulse, not a force
		joint.M_gamma = h * (d + h*k)
		if joint.M_gamma != 0.0 {
			joint.M_gamma = 1.0 / joint.M_gamma
		} else {
			joint.M_gamma = 0.0
		}
		joint.M_bias = C * h * k * joint.M_gamma

		invMass += joint.M_gamma
		if invMass != 0.0 {
			joint.M_softMass = 1.0 / invMass
		} else {
			joint.M_softMass = 0.0
		}
	} else {
		// rigid
		joint.M_gamma = 0.0
		joint.M_bias = 0.0
		joint.M_softMass = joint.M_mass
	}

	if data.Step.WarmStarting {
		// Scale the impulse to support a variable time step.
		joint.M_impulse *= data.Step.DtRatio
		joint.M_lowerImpulse *= data.Step.DtRatio
		joint.M_upperImpulse *= data.Step.DtRatio

		P := Vec2MulScalar(joint.M_impulse+joint.M_lowerImpulse-joint.M_upperImpulse, joint.M_u)
		vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
		wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
		vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
		wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)
	} else {
		joint.M_impulse = 0.0
	}

	// Note: mutation on value, not ref; but OK because Velocities is an array
	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *DistanceJoint) SolveVelocityConstraints(data SolverData) {
	vA := data.Velocities[joint.M_indexA].V
	wA := data.Velocities[joint.M_indexA].W
	vB := data.Velocities[joint.M_indexB].V
	wB := data.Velocities[joint.M_indexB].W

	if joint.M_minLength < joint.M_maxLength {
		if joint.M_stiffness > 0.0 {
			// Cdot = dot(u, v + cross(w, r))
			vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
			vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))
			Cdot := Vec2Dot(joint.M_u, Vec2Sub(vpB, vpA))

			impulse := -joint.M_softMass * (Cdot + joint.M_bias + joint.M_gamma*joint.M_impulse)
			joint.M_impulse += impulse

			P := Vec2MulScalar(impulse, joint.M_u)
			vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
			wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
			vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
			wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)
		}

		// lower
		{
			C := joint.M_currentLength - joint.M_minLength
			bias := math.Max(0.0, C) * data.Step.Inv_dt

			vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
			vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))
			Cdot := Vec2Dot(joint.M_u, Vec2Sub(vpB, vpA))

			impulse := -joint.M_mass * (Cdot + bias)
			oldImpulse := joint.M_lowerImpulse
			joint.M_lowerImpulse = math.Max(0.0, joint.M_lowerImpulse+impulse)
			impulse = joint.M_lowerImpulse - oldImpulse
			P := Vec2MulScalar(impulse, joint.M_u)

			vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
			wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
			vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
			wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)
		}

		// upper
		{
			C := joint.M_maxLength - joint.M_currentLength
			bias := math.Max(0.0, C) * data.Step.Inv_dt

			vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
			vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))
			Cdot := Vec2Dot(joint.M_u, Vec2Sub(vpA, vpB))

			impulse := -joint.M_mass * (Cdot + bias)
			oldImpulse := joint.M_upperImpulse
			joint.M_upperImpulse = math.Max(0.0, joint.M_upperImpulse+impulse)
			impulse = joint.M_upperImpulse - oldImpulse
			P := Vec2MulScalar(-impulse, joint.M_u)

			vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
			wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
			vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
			wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)
		}
	} else {
		// Equal limits

		// Cdot = dot(u, v + cross(w, r))
		vpA := Vec2Add(vA, Vec2CrossScalarVector(wA, joint.M_rA))
		vpB := Vec2Add(vB, Vec2CrossScalarVector(wB, joint.M_rB))
		Cdot := Vec2Dot(joint.M_u, Vec2Sub(vpB, vpA))

		impulse := -joint.M_mass * Cdot
		joint.M_impulse += impulse

		P := Vec2MulScalar(impulse, joint.M_u)
		vA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
		wA -= joint.M_invIA * Vec2Cross(joint.M_rA, P)
		vB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
		wB += joint.M_invIB * Vec2Cross(joint.M_rB, P)
	}

	// Note: mutation on value, not ref; but OK because Velocities is an array
	data.Velocities[joint.M_indexA].V = vA
	data.Velocities[joint.M_indexA].W = wA
	data.Velocities[joint.M_indexB].V = vB
	data.Velocities[joint.M_indexB].W = wB
}

func (joint *DistanceJoint) SolvePositionConstraints(data SolverData) bool {
	cA := data.Positions[joint.M_indexA].C
	aA := data.Positions[joint.M_indexA].A
	cB := data.Positions[joint.M_indexB].C
	aB := data.Positions[joint.M_indexB].A

	qA := MakeRotFromAngle(aA)
	qB := MakeRotFromAngle(aB)

	rA := RotVec2Mul(qA, Vec2Sub(joint.M_localAnchorA, joint.M_localCenterA))
	rB := RotVec2Mul(qB, Vec2Sub(joint.M_localAnchorB, joint.M_localCenterB))
	u := Vec2Sub(Vec2Sub(Vec2Add(cB, rB), cA), rA)

	length := u.Normalize()
	var C float64
	if joint.M_minLength == joint.M_maxLength {
		C = length - joint.M_minLength
	} else if length < joint.M_minLength {
		C = length - joint.M_minLength
	} else if joint.M_maxLength < length {
		C = length - joint.M_maxLength
	} else {
		return true
	}

	impulse := -joint.M_mass * C
	P := Vec2MulScalar(impulse, u)

	cA.OperatorMinusInplace(Vec2MulScalar(joint.M_invMassA, P))
	aA -= joint.M_invIA * Vec2Cross(rA, P)
	cB.OperatorPlusInplace(Vec2MulScalar(joint.M_invMassB, P))
	aB += joint.M_invIB * Vec2Cross(rB, P)

	// Note: mutation on value, not ref; but OK because Positions is an array
	data.Positions[joint.M_indexA].C = cA
	data.Positions[joint.M_indexA].A = aA
	data.Positions[joint.M_indexB].C = cB
	data.Positions[joint.M_indexB].A = aB

	return math.Abs(C) < linearSlop
}

func (joint DistanceJoint) GetAnchorA() Vec2 {
	return joint.bodyA.WorldPoint(joint.M_localAnchorA)
}

func (joint DistanceJoint) GetAnchorB() Vec2 {
	return joint.bodyB.WorldPoint(joint.M_localAnchorB)
}

func (joint DistanceJoint) GetReactionForce(inv_dt float64) Vec2 {
	return Vec2MulScalar(inv_dt*(joint.M_impulse+joint.M_lowerImpulse-joint.M_upperImpulse), joint.M_u)
}

func (joint DistanceJoint) GetReactionTorque(inv_dt float64) float64 {
	return 0.0
}

func (joint DistanceJoint) Dump() {
	indexA := joint.bodyA.islandIndex
	indexB := joint.bodyB.islandIndex

	fmt.Printf("  b2DistanceJointDef jd;\n")
	fmt.Printf("  jd.bodyA = bodies[%d];\n", indexA)
	fmt.Printf("  jd.bodyB = bodies[%d];\n", indexB)
	fmt.Printf("  jd.collideConnected = bool(%v);\n", joint.M_collideConnected)
	fmt.Printf("  jd.localAnchorA.Set(%.15f, %.15f);\n", joint.M_localAnchorA.X, joint.M_localAnchorA.Y)
	fmt.Printf("  jd.localAnchorB.Set(%.15f, %.15f);\n", joint.M_localAnchorB.X, joint.M_localAnchorB.Y)
	fmt.Printf("  jd.length = %.15f;\n", joint.M_length)
	fmt.Printf("  jd.minLength = %.15f;\n", joint.M_minLength)
	fmt.Printf("  jd.maxLength = %.15f;\n", joint.M_maxLength)
	fmt.Printf("  jd.frequencyHz = %.15f;\n", joint.M_stiffness)
	fmt.Printf("  jd.dampingRatio = %.15f;\n", joint.M_damping)
	fmt.Printf("  joints[%d] = m_world.CreateJoint(&jd);\n", joint.M_index)
}

//void b2DistanceJoint::Draw(b2Draw* draw) const
//{
//	const b2Transform& xfA = m_bodyA->GetTransform();
//	const b2Transform& xfB = m_bodyB->GetTransform();
//	b2Vec2 pA = b2Mul(xfA, m_localAnchorA);
//	b2Vec2 pB = b2Mul(xfB, m_localAnchorB);
//
//	b2Vec2 axis = pB - pA;
//	axis.Normalize();
//
//	b2Color c1(0.7f, 0.7f, 0.7f);
//	b2Color c2(0.3f, 0.9f, 0.3f);
//	b2Color c3(0.9f, 0.3f, 0.3f);
//	b2Color c4(0.4f, 0.4f, 0.4f);
//
//	draw->DrawSegment(pA, pB, c4);
//
//	b2Vec2 pRest = pA + m_length * axis;
//	draw->DrawPoint(pRest, 8.0f, c1);
//
//	if (m_minLength != m_maxLength)
//	{
//		if (m_minLength > b2_linearSlop)
//		{
//			b2Vec2 pMin = pA + m_minLength * axis;
//			draw->DrawPoint(pMin, 4.0f, c2);
//		}
//
//		if (m_maxLength < FLT_MAX)
//		{
//			b2Vec2 pMax = pA + m_maxLength * axis;
//			draw->DrawPoint(pMax, 4.0f, c3);
//		}
//	}
//}
