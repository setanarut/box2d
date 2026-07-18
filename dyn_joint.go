package b2

type JointType uint8

const (
	UnknownJointType JointType = iota // 0
	RevoluteJointType
	PrismaticJointType
	DistanceJointType
	PulleyJointType
	MouseJointType
	GearJointType
	WheelJointType
	WeldJointType
	FrictionJointType
	MotorJointType
)

type Jacobian struct {
	Linear   Vec2
	AngularA float64
	AngularB float64
}

// A joint edge is used to connect bodies and joints together
// in a joint graph where each body is a node and each joint
// is an edge. A joint edge belongs to a doubly linked list
// maintained in each attached body. Each joint has two joint
// nodes, one for each attached body.
type JointEdge struct {
	Other *Body      // provides quick access to the other body attached.
	Joint IJoint     // the joint; backed by pointer
	Prev  *JointEdge // the previous joint edge in the body's joint list
	Next  *JointEdge // the next joint edge in the body's joint list
}

// Joint definitions are used to construct joints.
type JointDef struct {

	// The joint type is set automatically for concrete joint types.
	Type JointType

	// Use this to attach application specific data to your joints.
	UserData any

	// The first attached body.
	BodyA *Body

	// The second attached body.
	BodyB *Body

	// Set this flag to true if the attached bodies should collide.
	CollideConnected bool
}

type IJointDef interface {
	GetType() JointType
	SetType(t JointType)
	GetUserData() any
	SetUserData(userData any)
	GetBodyA() *Body
	SetBodyA(body *Body)
	GetBodyB() *Body
	SetBodyB(body *Body)
	IsCollideConnected() bool
	SetCollideConnected(flag bool)
}

// Implementing JointDefInterface on Joint (used as a base struct)
func (def JointDef) GetType() JointType {
	return def.Type
}

func (def *JointDef) SetType(t JointType) {
	def.Type = t
}

func (def JointDef) GetUserData() any {
	return def.UserData
}

func (def *JointDef) SetUserData(userdata any) {
	def.UserData = userdata
}

func (def JointDef) GetBodyA() *Body {
	return def.BodyA
}

func (def *JointDef) SetBodyA(body *Body) {
	def.BodyA = body
}

func (def JointDef) GetBodyB() *Body {
	return def.BodyB
}

func (def *JointDef) SetBodyB(body *Body) {
	def.BodyB = body
}

func (def JointDef) IsCollideConnected() bool {
	return def.CollideConnected
}

func (def *JointDef) SetCollideConnected(flag bool) {
	def.CollideConnected = flag
}

func DefaultJointDef() JointDef {
	return JointDef{}
}

// Utility to compute linear stiffness values from frequency and damping ratio
func LinearStiffness(stiffness *float64, damping *float64, frequencyHertz float64, dampingRatio float64, bodyA *Body, bodyB *Body) {
	massA := bodyA.Mass()
	massB := bodyB.Mass()
	var mass float64
	if massA > 0.0 && massB > 0.0 {
		mass = massA * massB / (massA + massB)
	} else if massA > 0.0 {
		mass = massA
	} else {
		mass = massB
	}

	omega := 2.0 * pi * frequencyHertz
	*stiffness = mass * omega * omega
	*damping = 2.0 * mass * dampingRatio * omega
}

// Utility to compute rotational stiffness values frequency and damping ratio
func AngularStiffness(stiffness *float64, damping *float64, frequencyHertz float64, dampingRatio float64, bodyA *Body, bodyB *Body) {
	IA := bodyA.Inertia()
	IB := bodyB.Inertia()
	var I float64
	if IA > 0.0 && IB > 0.0 {
		I = IA * IB / (IA + IB)
	} else if IA > 0.0 {
		I = IA
	} else {
		I = IB
	}

	omega := 2.0 * pi * frequencyHertz
	*stiffness = I * omega * omega
	*damping = 2.0 * I * dampingRatio * omega
}

// The base joint class. Joints are used to constraint two bodies together in
// various fashions. Some joints also feature limits and motors.
type Joint struct {
	M_type             JointType
	M_prev             IJoint // has to be backed by pointer
	M_next             IJoint // has to be backed by pointer
	M_edgeA            *JointEdge
	M_edgeB            *JointEdge
	bodyA              *Body
	bodyB              *Body
	M_index            int
	M_islandFlag       bool
	M_collideConnected bool
	UserData           any
}

// Dump this joint to the log file.
func (j Joint) Dump() {}

// Shift the origin for any points stored in world coordinates.
func (j Joint) ShiftOrigin(newOrigin Vec2) {}

func (j Joint) GetType() JointType {
	return j.M_type
}

// @goadd
func (j *Joint) SetType(t JointType) {
	j.M_type = t
}

func (j Joint) GetBodyA() *Body {
	return j.bodyA
}

// @goadd
func (j *Joint) SetBodyA(body *Body) {
	j.bodyA = body
}

func (j Joint) GetBodyB() *Body {
	return j.bodyB
}

// @goadd
func (j *Joint) SetBodyB(body *Body) {
	j.bodyB = body
}

func (j Joint) GetNext() IJoint { // returns pointer
	return j.M_next
}

// @goadd
func (j *Joint) SetNext(next IJoint) { // has to be backed by pointer
	j.M_next = next
}

func (j Joint) GetPrev() IJoint { // returns pointer
	return j.M_prev
}

// @goadd
func (j *Joint) SetPrev(prev IJoint) { // prev has to be backed by pointer
	j.M_prev = prev
}

func (j Joint) GetUserData() any {
	return j.UserData
}

func (j *Joint) SetUserData(data any) {
	j.UserData = data
}

func (j Joint) IsCollideConnected() bool {
	return j.M_collideConnected
}

// @goadd
func (j *Joint) SetCollideConnected(flag bool) {
	j.M_collideConnected = flag
}

// @goadd
func (j Joint) GetEdgeA() *JointEdge {
	return j.M_edgeA
}

// @goadd
func (j *Joint) SetEdgeA(edge *JointEdge) {
	j.M_edgeA = edge
}

// @goadd
func (j Joint) GetEdgeB() *JointEdge {
	return j.M_edgeB
}

// @goadd
func (j *Joint) SetEdgeB(edge *JointEdge) {
	j.M_edgeB = edge
}

func JointCreate(def IJointDef) IJoint { // def should be back by pointer; a pointer is returned
	var joint *Joint = nil
	switch def.GetType() {
	case DistanceJointType:
		if typeddef, ok := def.(*DistanceJointDef); ok {
			return MakeDistanceJoint(typeddef)
		}
		assert(false)
	case MouseJointType:
		if typeddef, ok := def.(*MouseJointDef); ok {
			return MakeMouseJoint(typeddef)
		}
		assert(false)
	case PrismaticJointType:
		if typeddef, ok := def.(*PrismaticJointDef); ok {
			return MakePrismaticJoint(typeddef)
		}
		assert(false)
	case RevoluteJointType:
		if typeddef, ok := def.(*RevoluteJointDef); ok {
			return MakeRevoluteJoint(typeddef)
		}
		assert(false)
	case PulleyJointType:
		if typeddef, ok := def.(*PulleyJointDef); ok {
			return MakePulleyJoint(typeddef)
		}
		assert(false)
	case GearJointType:
		if typeddef, ok := def.(*GearJointDef); ok {
			return MakeGearJoint(typeddef)
		}
		assert(false)
	case WheelJointType:
		if typeddef, ok := def.(*WheelJointDef); ok {
			return MakeWheelJoint(typeddef)
		}
		assert(false)
	case WeldJointType:
		if typeddef, ok := def.(*WeldJointDef); ok {
			return MakeWeldJoint(typeddef)
		}
		assert(false)
	case FrictionJointType:
		if typeddef, ok := def.(*FrictionJointDef); ok {
			return MakeFrictionJoint(typeddef)
		}
		assert(false)
	case MotorJointType:
		if typeddef, ok := def.(*MotorJointDef); ok {
			return MakeMotorJoint(typeddef)
		}
		assert(false)
	default:
		assert(false)
	}
	return joint
}

func JointDestroy(joint IJoint) { // has to be backed by pointer
	joint.Destroy()
}

func MakeJoint(def IJointDef) *Joint { // def has to be backed by pointer
	assert(def.GetBodyA() != def.GetBodyB())

	res := Joint{}

	res.M_type = def.GetType()
	res.M_prev = nil
	res.M_next = nil
	res.bodyA = def.GetBodyA()
	res.bodyB = def.GetBodyB()
	res.M_index = 0
	res.M_collideConnected = def.IsCollideConnected()
	res.M_islandFlag = false
	res.UserData = def.GetUserData()

	res.M_edgeA = &JointEdge{}
	res.M_edgeB = &JointEdge{}

	return &res
}

// Short-cut function to determine if either body is enabled.
func (j Joint) IsEnabled() bool {
	return j.bodyA.IsEnabled() && j.bodyB.IsEnabled()
}

// @goadd
func (j *Joint) Destroy() {

}

// @goadd
func (j Joint) GetIndex() int {
	return j.M_index
}

func (j *Joint) SetIndex(index int) {
	j.M_index = index
}

func (j *Joint) InitVelocityConstraints(data SolverData) {}

func (j *Joint) SolveVelocityConstraints(data SolverData) {}

func (j *Joint) SolvePositionConstraints(data SolverData) bool {
	return false
}

func (j Joint) GetIslandFlag() bool {
	return j.M_islandFlag
}

func (j *Joint) SetIslandFlag(flag bool) {
	j.M_islandFlag = flag
}

type IJoint interface {
	// Dump this joint to the log file.
	Dump()

	// Shift the origin for any points stored in world coordinates.
	ShiftOrigin(newOrigin Vec2)

	GetType() JointType
	SetType(t JointType)

	GetBodyA() *Body
	SetBodyA(body *Body)

	GetBodyB() *Body
	SetBodyB(body *Body)

	GetIndex() int
	SetIndex(index int)

	GetNext() IJoint     // backed by pointer
	SetNext(next IJoint) // backed by pointer

	GetPrev() IJoint     // backed by pointer
	SetPrev(prev IJoint) // backed by pointer

	GetEdgeA() *JointEdge
	SetEdgeA(edge *JointEdge)

	GetEdgeB() *JointEdge
	SetEdgeB(edge *JointEdge)

	GetUserData() any
	SetUserData(data any)

	IsCollideConnected() bool
	SetCollideConnected(flag bool)

	IsEnabled() bool

	//@goadd
	Destroy()

	InitVelocityConstraints(data SolverData)

	SolveVelocityConstraints(data SolverData)

	SolvePositionConstraints(data SolverData) bool

	GetIslandFlag() bool
	SetIslandFlag(flag bool)
}
