package b2

import (
	"fmt"
)

type BodyType uint8

const (
	// Zero mass, zero velocity, may be manually moved
	Static BodyType = iota
	// Zero mass, non-zero velocity set by user, moved by solver
	Kinematic
	// Positive mass, non-zero velocity determined by forces, moved by solver
	Dynamic
)

// A body definition holds all the data needed to construct a rigid body.
// You can safely re-use body definitions. Shapes are added to a body after construction.
type BodyDef struct {

	// Note: if a dynamic body would have zero mass, the mass is set to one.
	Type BodyType

	// The world position of the body. Avoid creating bodies at the origin
	// since this can lead to many overlapping shapes.
	Position Vec2

	// The linear velocity of the body's origin in world co-ordinates.
	LinearVelocity Vec2

	// The world angle of the body in radians.
	Angle float64

	// The angular velocity of the body.
	AngularVelocity float64

	// Linear damping is use to reduce the linear velocity. The damping parameter
	// can be larger than 1.0 but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	// Units are 1/time
	LinearDamping float64

	// Angular damping is use to reduce the angular velocity. The damping parameter
	// can be larger than 1.0 but the damping effect becomes sensitive to the
	// time step when the damping parameter is large.
	// Units are 1/time
	AngularDamping float64

	// Set this flag to false if this body should never fall asleep. Note that
	// this increases CPU usage.
	AllowSleep bool

	// Is this body initially awake or sleeping?
	Awake bool

	// Should this body be prevented from rotating? Useful for characters.
	FixedRotation bool

	// Is this a fast moving body that should be prevented from tunneling through
	// other moving bodies? Note that all bodies are prevented from tunneling through
	// kinematic and static bodies. This setting is only considered on dynamic bodies.
	// @warning You should use this flag sparingly since it increases processing time.
	Bullet bool

	// Does this body start out enabled?
	Enabled bool

	// Scale the gravity applied to this body.
	GravityScale float64

	// Use this to store application specific body data.
	UserData any
}

// This constructor sets the body definition default values.
func DefaultBodyDef() BodyDef {
	return BodyDef{
		AllowSleep:   true,
		Awake:        true,
		Type:         Static,
		Enabled:      true,
		GravityScale: 1.0,
	}
}

func NewBodyDef() *BodyDef {
	res := DefaultBodyDef()
	return &res
}

const (
	BodyIslandFlag        uint32 = 0x0001
	BodyAwakeFlag         uint32 = 0x0002
	BodyAutoSleepFlag     uint32 = 0x0004
	BodyBulletFlag        uint32 = 0x0008
	BodyFixedRotationFlag uint32 = 0x0010
	BodyEnabledFlag       uint32 = 0x0020
	BodyToiFlag           uint32 = 0x0040
)

type Body struct {
	bodyType BodyType
	flags    uint32

	islandIndex int

	xf    Transform // the body origin transform
	sweep Sweep     // the swept motion for CCD

	linearVelocity  Vec2
	angularVelocity float64

	force  Vec2
	torque float64

	world *World
	prev  *Body
	next  *Body

	fixtureList  *Fixture // linked list
	fixtureCount int

	jointList   *JointEdge   // linked list
	contactList *ContactEdge // linked list

	mass, invMass float64

	// Rotational inertia about the center of mass.
	inertia, invInertia float64

	linearDamping  float64
	angularDamping float64
	gravityScale   float64

	sleepTime float64

	UserData any
}

func (body Body) Type() BodyType {
	return body.bodyType
}

func (body Body) IsDynamic() bool {
	return body.bodyType == Dynamic
}
func (body Body) IsStatic() bool {
	return body.bodyType == Static
}
func (body Body) IsKinematic() bool {
	return body.bodyType == Kinematic
}

func (body Body) Transform() Transform {
	return body.xf
}

func (body Body) Position() Vec2 {
	return body.xf.P
}

func (body Body) Angle() float64 {
	return body.sweep.A
}

func (body Body) WorldCenter() Vec2 {
	return body.sweep.C
}

func (body Body) LocalCenter() Vec2 {
	return body.sweep.LocalCenter
}

func (body *Body) SetLinearVelocity(v Vec2) {
	if body.bodyType == Static {
		return
	}

	if Vec2Dot(v, v) > 0.0 {
		body.SetAwake(true)
	}

	body.linearVelocity = v
}

func (body Body) LinearVelocity() Vec2 {
	return body.linearVelocity
}

func (body *Body) SetAngularVelocity(w float64) {
	if body.bodyType == Static {
		return
	}

	if w*w > 0.0 {
		body.SetAwake(true)
	}

	body.angularVelocity = w
}

func (body Body) GetAngularVelocity() float64 {
	return body.angularVelocity
}

func (body Body) Mass() float64 {
	return body.mass
}

func (body Body) Inertia() float64 {
	return body.inertia + body.mass*Vec2Dot(body.sweep.LocalCenter, body.sweep.LocalCenter)
}

func (body Body) MassData() MassData {
	var data MassData
	data.Mass = body.mass
	data.I = body.inertia + body.mass*Vec2Dot(body.sweep.LocalCenter, body.sweep.LocalCenter)
	data.Center = body.sweep.LocalCenter
	return data
}

func (body Body) WorldPoint(localPoint Vec2) Vec2 {
	return TransformVec2Mul(body.xf, localPoint)
}

func (body Body) WorldVector(localVector Vec2) Vec2 {
	return RotVec2Mul(body.xf.Q, localVector)
}

// Gets a local point relative to the body's origin given a world point.
func (body Body) LocalPoint(worldPoint Vec2) Vec2 {
	return TransformVec2MulT(body.xf, worldPoint)
}

func (body Body) LocalVector(worldVector Vec2) Vec2 {
	return RotVec2MulT(body.xf.Q, worldVector)
}

func (body Body) LinearVelocityFromWorldPoint(worldPoint Vec2) Vec2 {
	return Vec2Add(body.linearVelocity, Vec2CrossScalarVector(body.angularVelocity, Vec2Sub(worldPoint, body.sweep.C)))
}

func (body Body) LinearVelocityFromLocalPoint(localPoint Vec2) Vec2 {
	return body.LinearVelocityFromWorldPoint(body.WorldPoint(localPoint))
}

func (body Body) LinearDamping() float64 {
	return body.linearDamping
}

func (body *Body) SetLinearDamping(linearDamping float64) {
	body.linearDamping = linearDamping
}

func (body Body) GetAngularDamping() float64 {
	return body.angularDamping
}

func (body *Body) SetAngularDamping(angularDamping float64) {
	body.angularDamping = angularDamping
}

func (body Body) GravityScale() float64 {
	return body.gravityScale
}

func (body *Body) SetGravityScale(scale float64) {
	body.gravityScale = scale
}

func (body *Body) SetBullet(flag bool) {
	if flag {
		body.flags |= BodyBulletFlag
	} else {
		body.flags &= ^BodyBulletFlag
	}
}

func (body Body) IsBullet() bool {
	return (body.flags & BodyBulletFlag) == BodyBulletFlag
}

func (body *Body) SetAwake(flag bool) {
	if body.bodyType == Static {
		return
	}

	if flag {
		body.flags |= BodyAwakeFlag
		body.sleepTime = 0.0
	} else {
		body.flags &= ^BodyAwakeFlag
		body.sleepTime = 0.0
		body.linearVelocity.SetZero()
		body.angularVelocity = 0.0
		body.force.SetZero()
		body.torque = 0.0
	}
}

func (body Body) IsAwake() bool {
	return (body.flags & BodyAwakeFlag) == BodyAwakeFlag
}

func (body Body) IsEnabled() bool {
	return (body.flags & BodyEnabledFlag) == BodyEnabledFlag
}

func (body Body) IsFixedRotation() bool {
	return (body.flags & BodyFixedRotationFlag) == BodyFixedRotationFlag
}

func (body *Body) SetSleepingAllowed(flag bool) {
	if flag {
		body.flags |= BodyAutoSleepFlag
	} else {
		body.flags &= ^BodyAutoSleepFlag
		body.SetAwake(true)
	}
}

func (body Body) IsSleepingAllowed() bool {
	return (body.flags & BodyAutoSleepFlag) == BodyAutoSleepFlag
}

func (body Body) FixtureList() *Fixture {
	return body.fixtureList
}

func (body Body) JointList() *JointEdge {
	return body.jointList
}

func (body Body) ContactList() *ContactEdge {
	return body.contactList
}

func (body Body) Next() *Body {
	return body.next
}

func (body *Body) ApplyForce(force Vec2, point Vec2, wake bool) {
	if body.bodyType != Dynamic {
		return
	}

	if wake && (body.flags&BodyAwakeFlag) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate a force if the body is sleeping.
	if (body.flags & BodyAwakeFlag) != 0x0000 {
		body.force.OperatorPlusInplace(force)
		body.torque += Vec2Cross(
			Vec2Sub(point, body.sweep.C),
			force,
		)
	}
}

func (body *Body) ApplyForceToCenter(force Vec2, wake bool) {
	if body.bodyType != Dynamic {
		return
	}

	if wake && (body.flags&BodyAwakeFlag) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate a force if the body is sleeping
	if (body.flags & BodyAwakeFlag) != 0x0000 {
		body.force.OperatorPlusInplace(force)
	}
}

func (body *Body) ApplyTorque(torque float64, wake bool) {
	if body.bodyType != Dynamic {
		return
	}

	if wake && (body.flags&BodyAwakeFlag) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate a force if the body is sleeping
	if (body.flags & BodyAwakeFlag) != 0x0000 {
		body.torque += torque
	}
}

func (body *Body) ApplyLinearImpulse(impulse Vec2, point Vec2, wake bool) {
	if body.bodyType != Dynamic {
		return
	}

	if wake && (body.flags&BodyAwakeFlag) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate velocity if the body is sleeping
	if (body.flags & BodyAwakeFlag) != 0x0000 {
		body.linearVelocity.OperatorPlusInplace(Vec2MulScalar(body.invMass, impulse))
		body.angularVelocity += body.invInertia * Vec2Cross(
			Vec2Sub(point, body.sweep.C),
			impulse,
		)
	}
}

func (body *Body) ApplyLinearImpulseToCenter(impulse Vec2, wake bool) {
	if body.bodyType != Dynamic {
		return
	}

	if wake && (body.flags&BodyAwakeFlag) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate velocity if the body is sleeping
	if (body.flags & BodyAwakeFlag) != 0x0000 {
		body.linearVelocity.OperatorPlusInplace(Vec2MulScalar(body.invMass, impulse))
	}
}

func (body *Body) ApplyAngularImpulse(impulse float64, wake bool) {
	if body.bodyType != Dynamic {
		return
	}

	if wake && (body.flags&BodyAwakeFlag) == 0 {
		body.SetAwake(true)
	}

	// Don't accumulate velocity if the body is sleeping
	if (body.flags & BodyAwakeFlag) != 0x0000 {
		body.angularVelocity += body.invInertia * impulse
	}
}

func (body *Body) SynchronizeTransform() {
	body.xf.Q.Set(body.sweep.A)
	body.xf.P = Vec2Sub(body.sweep.C, RotVec2Mul(body.xf.Q, body.sweep.LocalCenter))
}

func (body *Body) Advance(alpha float64) {
	// Advance to the new safe time. This doesn't sync the broad-phase.
	body.sweep.Advance(alpha)
	body.sweep.C = body.sweep.C0
	body.sweep.A = body.sweep.A0
	body.xf.Q.Set(body.sweep.A)
	body.xf.P = Vec2Sub(body.sweep.C, RotVec2Mul(body.xf.Q, body.sweep.LocalCenter))
}

func (body Body) GetWorld() *World {
	return body.world
}

func NewBody(bd *BodyDef, world *World) *Body {
	assert(bd.Position.IsValid())
	assert(bd.LinearVelocity.IsValid())
	assert(IsValid(bd.Angle))
	assert(IsValid(bd.AngularVelocity))
	assert(IsValid(bd.AngularDamping) && bd.AngularDamping >= 0.0)
	assert(IsValid(bd.LinearDamping) && bd.LinearDamping >= 0.0)

	body := &Body{
		world:           world,
		linearVelocity:  bd.LinearVelocity,
		angularVelocity: bd.AngularVelocity,
		linearDamping:   bd.LinearDamping,
		angularDamping:  bd.AngularDamping,
		gravityScale:    bd.GravityScale,
		bodyType:        bd.Type,
		UserData:        bd.UserData,
	}

	if bd.Bullet {
		body.flags |= BodyBulletFlag
	}
	if bd.FixedRotation {
		body.flags |= BodyFixedRotationFlag
	}
	if bd.AllowSleep {
		body.flags |= BodyAutoSleepFlag
	}
	if bd.Awake && bd.Type != Static {
		body.flags |= BodyAwakeFlag
	}
	if bd.Enabled {
		body.flags |= BodyEnabledFlag
	}

	body.xf.P = bd.Position
	body.xf.Q.Set(bd.Angle)

	body.sweep.C0 = bd.Position
	body.sweep.C = bd.Position
	body.sweep.A0 = bd.Angle
	body.sweep.A = bd.Angle

	return body
}

func (body *Body) SetType(bt BodyType) {

	assert(body.world.IsLocked() == false)
	if body.world.IsLocked() == true {
		return
	}

	if body.bodyType == bt {
		return
	}

	body.bodyType = bt

	body.ResetMassData()

	if body.bodyType == Static {
		body.linearVelocity.SetZero()
		body.angularVelocity = 0.0
		body.sweep.A0 = body.sweep.A
		body.sweep.C0 = body.sweep.C
		body.flags &= ^BodyAwakeFlag
		body.SynchronizeFixtures()
	}

	body.SetAwake(true)

	body.force.SetZero()
	body.torque = 0.0

	// Delete the attached contacts.
	ce := body.contactList
	for ce != nil {
		ce0 := ce
		ce = ce.Next
		body.world.contactManager.Destroy(ce0.Contact)
	}

	body.contactList = nil

	// Touch the proxies so that new contacts will be created (when appropriate)
	broadPhase := body.world.contactManager.broadPhase
	for f := body.fixtureList; f != nil; f = f.next {
		proxyCount := f.proxyCount
		for i := range proxyCount {
			broadPhase.TouchProxy(f.proxies[i].ProxyId)
		}
	}
}

func (body *Body) CreateFixtureFromDef(def *FixtureDef) *Fixture {

	assert(body.world.IsLocked() == false)
	if body.world.IsLocked() == true {
		return nil
	}

	fixture := NewFixture()
	fixture.Create(body, def)

	if (body.flags & BodyEnabledFlag) != 0x0000 {
		broadPhase := &body.world.contactManager.broadPhase
		fixture.CreateProxies(broadPhase, body.xf)
	}

	fixture.next = body.fixtureList
	body.fixtureList = fixture
	body.fixtureCount++

	fixture.body = body

	// Adjust mass properties if needed.
	if fixture.density > 0.0 {
		body.ResetMassData()
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	body.world.newContacts = true

	return fixture
}

func (body *Body) CreateFixture(shape IShape, density float64) *Fixture {

	def := DefaultFixtureDef()
	def.Shape = shape
	def.Density = density

	return body.CreateFixtureFromDef(&def)
}

func (body *Body) DestroyFixture(fixture *Fixture) {

	if fixture == nil {
		return
	}

	assert(body.world.IsLocked() == false)
	if body.world.IsLocked() == true {
		return
	}

	assert(fixture.body == body)

	// Remove the fixture from this body's singly linked list.
	assert(body.fixtureCount > 0)
	node := &body.fixtureList
	found := false
	for *node != nil {
		if *node == fixture {
			*node = fixture.next
			found = true
			break
		}

		node = &(*node).next
	}

	// You tried to remove a shape that is not attached to this body.
	assert(found)

	density := fixture.density

	// Destroy any contacts associated with the fixture.
	edge := body.contactList
	for edge != nil {
		c := edge.Contact
		edge = edge.Next

		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()

		if fixture == fixtureA || fixture == fixtureB {
			// This destroys the contact and removes it from
			// this body's contact list.
			body.world.contactManager.Destroy(c)
		}
	}

	if (body.flags & BodyEnabledFlag) != 0x0000 {
		broadPhase := &body.world.contactManager.broadPhase
		fixture.DestroyProxies(broadPhase)
	}

	fixture.body = nil
	fixture.next = nil
	fixture.Destroy()

	body.fixtureCount--

	// Reset the mass data.
	if density > 0.0 {
		body.ResetMassData()
	}
}

func (body *Body) ResetMassData() {

	// Compute mass data from shapes. Each shape has its own density.
	body.mass = 0.0
	body.invMass = 0.0
	body.inertia = 0.0
	body.invInertia = 0.0
	body.sweep.LocalCenter.SetZero()

	// Static and kinematic bodies have zero mass.
	if body.bodyType == Static || body.bodyType == Kinematic {
		body.sweep.C0 = body.xf.P
		body.sweep.C = body.xf.P
		body.sweep.A0 = body.sweep.A
		return
	}

	assert(body.bodyType == Dynamic)

	// Accumulate mass over all fixtures.
	localCenter := Vec2{}
	for f := body.fixtureList; f != nil; f = f.next {
		if f.density == 0.0 {
			continue
		}

		massData := NewMassData()
		f.MassData(massData)
		body.mass += massData.Mass
		localCenter.OperatorPlusInplace(Vec2MulScalar(massData.Mass, massData.Center))
		body.inertia += massData.I
	}

	// Compute center of mass.
	if body.mass > 0.0 {
		body.invMass = 1.0 / body.mass
		localCenter.OperatorScalarMulInplace(body.invMass)
	}

	if body.inertia > 0.0 && (body.flags&BodyFixedRotationFlag) == 0 {
		// Center the inertia about the center of mass.
		body.inertia -= body.mass * Vec2Dot(localCenter, localCenter)
		assert(body.inertia > 0.0)
		body.invInertia = 1.0 / body.inertia

	} else {
		body.inertia = 0.0
		body.invInertia = 0.0
	}

	// Move center of mass.
	oldCenter := body.sweep.C
	body.sweep.LocalCenter = localCenter
	body.sweep.C0 = TransformVec2Mul(body.xf, body.sweep.LocalCenter)
	body.sweep.C = TransformVec2Mul(body.xf, body.sweep.LocalCenter)

	// Update center of mass velocity.
	body.linearVelocity.OperatorPlusInplace(Vec2CrossScalarVector(
		body.angularVelocity,
		Vec2Sub(body.sweep.C, oldCenter),
	))
}

func (body *Body) SetMassData(massData *MassData) {

	assert(body.world.IsLocked() == false)
	if body.world.IsLocked() == true {
		return
	}

	if body.bodyType != Dynamic {
		return
	}

	body.invMass = 0.0
	body.inertia = 0.0
	body.invInertia = 0.0

	body.mass = massData.Mass
	if body.mass <= 0.0 {
		body.mass = 1.0
	}

	body.invMass = 1.0 / body.mass

	if massData.I > 0.0 && (body.flags&BodyFixedRotationFlag) == 0 {
		body.inertia = massData.I - body.mass*Vec2Dot(massData.Center, massData.Center)
		assert(body.inertia > 0.0)
		body.invInertia = 1.0 / body.inertia
	}

	// Move center of mass.
	oldCenter := body.sweep.C
	body.sweep.LocalCenter = massData.Center
	body.sweep.C0 = TransformVec2Mul(body.xf, body.sweep.LocalCenter)
	body.sweep.C = TransformVec2Mul(body.xf, body.sweep.LocalCenter)

	// Update center of mass velocity.
	body.linearVelocity.OperatorPlusInplace(
		Vec2CrossScalarVector(
			body.angularVelocity,
			Vec2Sub(body.sweep.C, oldCenter),
		),
	)
}

func (body Body) ShouldCollide(other *Body) bool {

	// At least one body should be dynamic.
	if body.bodyType != Dynamic && other.bodyType != Dynamic {
		return false
	}

	// Does a joint prevent collision?
	for jn := body.jointList; jn != nil; jn = jn.Next {
		if jn.Other == other {
			if jn.Joint.IsCollideConnected() == false {
				return false
			}
		}
	}

	return true
}

func (body *Body) SetTransform(position Vec2, angle float64) {
	assert(body.world.IsLocked() == false)

	if body.world.IsLocked() == true {
		return
	}

	body.xf.Q.Set(angle)
	body.xf.P = position

	body.sweep.C = TransformVec2Mul(body.xf, body.sweep.LocalCenter)
	body.sweep.A = angle

	body.sweep.C0 = body.sweep.C
	body.sweep.A0 = angle

	broadPhase := &body.world.contactManager.broadPhase
	for f := body.fixtureList; f != nil; f = f.next {
		f.Synchronize(broadPhase, body.xf, body.xf)
	}

	// Check for new contacts the next step
	body.world.newContacts = true
}

func (body *Body) SynchronizeFixtures() {
	broadPhase := &body.world.contactManager.broadPhase

	if (body.flags & BodyAwakeFlag) != 0x0000 {
		xf1 := MakeTransform()
		xf1.Q.Set(body.sweep.A0)
		xf1.P = Vec2Sub(body.sweep.C0, RotVec2Mul(xf1.Q, body.sweep.LocalCenter))

		for f := body.fixtureList; f != nil; f = f.next {
			f.Synchronize(broadPhase, xf1, body.xf)
		}
	} else {
		for f := body.fixtureList; f != nil; f = f.next {
			f.Synchronize(broadPhase, body.xf, body.xf)
		}
	}
}

func (body *Body) SetActive(flag bool) {
	assert(body.world.IsLocked() == false)

	if flag == body.IsEnabled() {
		return
	}

	if flag {
		body.flags |= BodyEnabledFlag

		// Create all proxies.
		broadPhase := &body.world.contactManager.broadPhase
		for f := body.fixtureList; f != nil; f = f.next {
			f.CreateProxies(broadPhase, body.xf)
		}

		// Contacts are created at the beginning of the next
		body.world.newContacts = true
	} else {
		body.flags &= ^BodyEnabledFlag

		// Destroy all proxies.
		broadPhase := &body.world.contactManager.broadPhase
		for f := body.fixtureList; f != nil; f = f.next {
			f.DestroyProxies(broadPhase)
		}

		// Destroy the attached contacts.
		ce := body.contactList
		for ce != nil {
			ce0 := ce
			ce = ce.Next
			body.world.contactManager.Destroy(ce0.Contact)
		}

		body.contactList = nil
	}
}

func (body *Body) SetFixedRotation(flag bool) {
	status := (body.flags & BodyFixedRotationFlag) == BodyFixedRotationFlag

	if status == flag {
		return
	}

	if flag {
		body.flags |= BodyFixedRotationFlag
	} else {
		body.flags &= ^BodyFixedRotationFlag
	}

	body.angularVelocity = 0.0

	body.ResetMassData()
}

func (body *Body) Dump() {
	bodyIndex := body.islandIndex

	fmt.Print("{\n")
	fmt.Print("  b2BodyDef bd;\n")
	fmt.Print(fmt.Printf("  bd.type = b2BodyType(%d);\n", body.bodyType))
	fmt.Print(fmt.Printf("  bd.position.Set(%.15f, %.15f);\n", body.xf.P.X, body.xf.P.Y))
	fmt.Print(fmt.Printf("  bd.angle = %.15f;\n", body.sweep.A))
	fmt.Print(fmt.Printf("  bd.linearVelocity.Set(%.15f, %.15f);\n", body.linearVelocity.X, body.linearVelocity.Y))
	fmt.Print(fmt.Printf("  bd.angularVelocity = %.15f;\n", body.angularVelocity))
	fmt.Print(fmt.Printf("  bd.linearDamping = %.15f;\n", body.linearDamping))
	fmt.Print(fmt.Printf("  bd.angularDamping = %.15f;\n", body.angularDamping))
	fmt.Print(fmt.Printf("  bd.allowSleep = bool(%d);\n", body.flags&BodyAutoSleepFlag))
	fmt.Print(fmt.Printf("  bd.awake = bool(%d);\n", body.flags&BodyAwakeFlag))
	fmt.Print(fmt.Printf("  bd.fixedRotation = bool(%d);\n", body.flags&BodyFixedRotationFlag))
	fmt.Print(fmt.Printf("  bd.bullet = bool(%d);\n", body.flags&BodyBulletFlag))
	fmt.Print(fmt.Printf("  bd.active = bool(%d);\n", body.flags&BodyEnabledFlag))
	fmt.Print(fmt.Printf("  bd.gravityScale = %.15f;\n", body.gravityScale))
	fmt.Print(fmt.Printf("  bodies[%d] = body.M_world.CreateBody(&bd);\n", body.islandIndex))
	fmt.Print("\n")
	for f := body.fixtureList; f != nil; f = f.next {
		fmt.Print("  {\n")
		f.Dump(bodyIndex)
		fmt.Print("  }\n")
	}
	fmt.Print("}\n")
}
