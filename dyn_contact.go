package b2

import (
	"math"
)

// Friction mixing law. The idea is to allow either fixture to drive the friction to zero.
// For example, anything slides on ice.
func MixFriction(friction1, friction2 float64) float64 {
	return math.Sqrt(friction1 * friction2)
}

// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
// For example, a superball bounces on anything.
func MixRestitution(restitution1, restitution2 float64) float64 {
	if restitution1 > restitution2 {
		return restitution1
	}

	return restitution2
}

// Restitution mixing law. This picks the lowest value.
func MixRestitutionThreshold(threshold1, threshold2 float64) float64 {
	if threshold1 < threshold2 {
		return threshold1
	}

	return threshold2
}

type ContactCreateFcn func(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact // returned contact should be a pointer
type ContactDestroyFcn func(contact IContact)                                                     // contact should be a pointer

type ContactRegister struct {
	CreateFcn  ContactCreateFcn
	DestroyFcn ContactDestroyFcn
	Primary    bool
}

// A contact edge is used to connect bodies and contacts together
// in a contact graph where each body is a node and each contact
// is an edge. A contact edge belongs to a doubly linked list
// maintained in each attached body. Each contact has two contact
// nodes, one for each attached body.
type ContactEdge struct {
	Other   *Body        // provides quick access to the other body attached.
	Contact IContact     // the contact
	Prev    *ContactEdge // the previous contact edge in the body's contact list
	Next    *ContactEdge // the next contact edge in the body's contact list
}

func NewContactEdge() *ContactEdge {
	return &ContactEdge{}
}

const (
	ContactIslandFlag    uint32 = 0x0001
	ContactTouchingFlag  uint32 = 0x0002
	ContactEnabledFlag   uint32 = 0x0004
	ContactFilterFlag    uint32 = 0x0008
	ContactBulletHitFlag uint32 = 0x0010
	ContactToiFlag       uint32 = 0x0020
)

// The class manages contact between two shapes. A contact exists for each overlapping
// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
// that has no contact points.
var s_registers [][]ContactRegister
var s_initialized = false

type IContact interface {
	GetFlags() uint32
	SetFlags(flags uint32)

	GetPrev() IContact
	SetPrev(prev IContact)

	GetNext() IContact
	SetNext(prev IContact)

	GetNodeA() *ContactEdge
	SetNodeA(node *ContactEdge)

	GetNodeB() *ContactEdge
	SetNodeB(node *ContactEdge)

	GetFixtureA() *Fixture
	SetFixtureA(fixture *Fixture)

	GetFixtureB() *Fixture
	SetFixtureB(fixture *Fixture)

	GetChildIndexA() int
	SetChildIndexA(index int)

	GetChildIndexB() int
	SetChildIndexB(index int)

	GetManifold() *Manifold
	SetManifold(manifold *Manifold)

	GetTOICount() int
	SetTOICount(toiCount int)

	GetTOI() float64
	SetTOI(toiCount float64)

	GetFriction() float64
	SetFriction(friction float64)
	ResetFriction()

	GetRestitution() float64
	SetRestitution(restitution float64)
	ResetRestitution()
	SetRestitutionThreshold(float64)
	GetRestitutionThreshold() float64
	ResetRestitutionThreshold()

	GetTangentSpeed() float64
	SetTangentSpeed(tangentSpeed float64)

	IsTouching() bool
	IsEnabled() bool
	SetEnabled(bool)

	Evaluate(manifold *Manifold, xfA Transform, xfB Transform)

	FlagForFiltering()

	GetWorldManifold(worldManifold *WorldManifold)
}

type Contact struct {
	M_flags uint32

	// World pool and list pointers.
	M_prev IContact //should be backed by a pointer
	M_next IContact //should be backed by a pointer

	// Nodes for connecting bodies.
	M_nodeA *ContactEdge
	M_nodeB *ContactEdge

	M_fixtureA *Fixture
	M_fixtureB *Fixture

	M_indexA int
	M_indexB int

	M_manifold *Manifold

	M_toiCount             int
	M_toi                  float64
	M_friction             float64
	M_restitution          float64
	M_restitutionThreshold float64

	M_tangentSpeed float64
}

func (contact Contact) GetFlags() uint32 {
	return contact.M_flags
}

func (contact *Contact) SetFlags(flags uint32) {
	contact.M_flags = flags
}

func (contact Contact) GetPrev() IContact {
	return contact.M_prev
}

func (contact *Contact) SetPrev(prev IContact) {
	contact.M_prev = prev
}

func (contact Contact) GetNext() IContact {
	return contact.M_next
}

func (contact *Contact) SetNext(next IContact) {
	contact.M_next = next
}

func (contact Contact) GetNodeA() *ContactEdge {
	return contact.M_nodeA
}

func (contact *Contact) SetNodeA(node *ContactEdge) {
	contact.M_nodeA = node
}

func (contact Contact) GetNodeB() *ContactEdge {
	return contact.M_nodeB
}

func (contact *Contact) SetNodeB(node *ContactEdge) {
	contact.M_nodeB = node
}

func (contact Contact) GetFixtureA() *Fixture {
	return contact.M_fixtureA
}

func (contact *Contact) SetFixtureA(fixture *Fixture) {
	contact.M_fixtureA = fixture
}

func (contact Contact) GetFixtureB() *Fixture {
	return contact.M_fixtureB
}

func (contact *Contact) SetFixtureB(fixture *Fixture) {
	contact.M_fixtureB = fixture
}

func (contact Contact) GetChildIndexA() int {
	return contact.M_indexA
}

func (contact *Contact) SetChildIndexA(index int) {
	contact.M_indexA = index
}

func (contact Contact) GetChildIndexB() int {
	return contact.M_indexB
}

func (contact *Contact) SetChildIndexB(index int) {
	contact.M_indexB = index
}

func (contact Contact) GetManifold() *Manifold {
	return contact.M_manifold
}

func (contact *Contact) SetManifold(manifold *Manifold) {
	contact.M_manifold = manifold
}

func (contact Contact) GetTOICount() int {
	return contact.M_toiCount
}

func (contact *Contact) SetTOICount(toiCount int) {
	contact.M_toiCount = toiCount
}

func (contact Contact) GetTOI() float64 {
	return contact.M_toi
}

func (contact *Contact) SetTOI(toi float64) {
	contact.M_toi = toi
}

// Get the friction.
func (contact Contact) GetFriction() float64 {
	return contact.M_friction
}

// Override the default friction mixture. You can call this in b2ContactListener::PreSolve.
// This value persists until set or reset.
func (contact *Contact) SetFriction(friction float64) {
	contact.M_friction = friction
}

// Reset the friction mixture to the default value.
func (contact *Contact) ResetFriction() {
	contact.M_friction = MixFriction(contact.M_fixtureA.friction, contact.M_fixtureB.friction)
}

// Get the restitution.
func (contact Contact) GetRestitution() float64 {
	return contact.M_restitution
}

// Override the default restitution mixture. You can call this in b2ContactListener::PreSolve.
// The value persists until you set or reset.
func (contact *Contact) SetRestitution(restitution float64) {
	contact.M_restitution = restitution
}

// Reset the restitution to the default value.
func (contact *Contact) ResetRestitution() {
	contact.M_restitution = MixRestitution(contact.M_fixtureA.restitution, contact.M_fixtureB.restitution)
}

// Override the default restitution velocity threshold mixture. You can call this in b2ContactListener::PreSolve.
// The value persists until you set or reset.
func (contact *Contact) SetRestitutionThreshold(threshold float64) {
	contact.M_restitutionThreshold = threshold
}

// Get the restitution threshold.
func (contact Contact) GetRestitutionThreshold() float64 {
	return contact.M_restitutionThreshold
}

// Reset the restitution threshold to the default value.
func (contact *Contact) ResetRestitutionThreshold() {
	contact.M_restitutionThreshold = MixRestitutionThreshold(contact.M_fixtureA.restitutionThreshold, contact.M_fixtureB.restitutionThreshold)
}

// Get the desired tangent speed. In meters per second.
func (contact Contact) GetTangentSpeed() float64 {
	return contact.M_tangentSpeed
}

// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
func (contact *Contact) SetTangentSpeed(speed float64) {
	contact.M_tangentSpeed = speed
}

func (contact Contact) GetWorldManifold(worldManifold *WorldManifold) {
	bodyA := contact.M_fixtureA.Body()
	bodyB := contact.M_fixtureB.Body()
	shapeA := contact.M_fixtureA.Shape()
	shapeB := contact.M_fixtureB.Shape()

	worldManifold.Initialize(contact.M_manifold, bodyA.Transform(), shapeA.GetRadius(), bodyB.Transform(), shapeB.GetRadius())
}

func (contact *Contact) SetEnabled(flag bool) {
	if flag {
		contact.M_flags |= ContactEnabledFlag
	} else {
		contact.M_flags &= ^ContactEnabledFlag
	}
}

func (contact Contact) IsEnabled() bool {
	return (contact.M_flags & ContactEnabledFlag) == ContactEnabledFlag
}

func (contact Contact) IsTouching() bool {
	return (contact.M_flags & ContactTouchingFlag) == ContactTouchingFlag
}

func (contact *Contact) FlagForFiltering() {
	contact.M_flags |= ContactFilterFlag
}

func ContactInitializeRegisters() {
	s_registers = make([][]ContactRegister, typeCount)
	for i := 0; i < int(typeCount); i++ {
		s_registers[i] = make([]ContactRegister, typeCount)
	}

	AddType(CircleContact_Create, CircleContact_Destroy, Circle, Circle)
	AddType(PolygonAndCircleContact_Create, PolygonAndCircleContact_Destroy, Polygon, Circle)
	AddType(PolygonContact_Create, PolygonContact_Destroy, Polygon, Polygon)
	AddType(EdgeAndCircleContact_Create, EdgeAndCircleContact_Destroy, Edge, Circle)
	AddType(EdgeAndPolygonContact_Create, EdgeAndPolygonContact_Destroy, Edge, Polygon)
	AddType(ChainAndCircleContact_Create, ChainAndCircleContact_Destroy, Chain, Circle)
	AddType(ChainAndPolygonContact_Create, ChainAndPolygonContact_Destroy, Chain, Polygon)
}

func AddType(createFcn ContactCreateFcn, destroyFcn ContactDestroyFcn, type1, type2 ShapeType) {
	assert(type1 < typeCount)
	assert(type2 < typeCount)

	s_registers[type1][type2].CreateFcn = createFcn
	s_registers[type1][type2].DestroyFcn = destroyFcn
	s_registers[type1][type2].Primary = true

	if type1 != type2 {
		s_registers[type2][type1].CreateFcn = createFcn
		s_registers[type2][type1].DestroyFcn = destroyFcn
		s_registers[type2][type1].Primary = false
	}
}

func ContactFactory(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact { // returned contact should be a pointer

	if s_initialized == false {
		ContactInitializeRegisters()
		s_initialized = true
	}

	type1 := fixtureA.Type()
	type2 := fixtureB.Type()

	assert(type1 < typeCount)
	assert(type2 < typeCount)

	createFcn := s_registers[type1][type2].CreateFcn
	if createFcn != nil {
		if s_registers[type1][type2].Primary {
			return createFcn(fixtureA, indexA, fixtureB, indexB)
		} else {
			return createFcn(fixtureB, indexB, fixtureA, indexA)
		}
	}

	return nil
}

func ContactDestroy(contact IContact) {
	assert(s_initialized == true)

	fixtureA := contact.GetFixtureA()
	fixtureB := contact.GetFixtureB()

	if contact.GetManifold().PointCount > 0 && fixtureA.IsSensor() == false && fixtureB.IsSensor() == false {
		fixtureA.Body().SetAwake(true)
		fixtureB.Body().SetAwake(true)
	}

	typeA := fixtureA.Type()
	typeB := fixtureB.Type()

	assert(typeA < typeCount)
	assert(typeB < typeCount)

	destroyFcn := s_registers[typeA][typeB].DestroyFcn
	destroyFcn(contact)
}

func MakeContact(fA *Fixture, indexA int, fB *Fixture, indexB int) Contact {
	contact := Contact{}
	contact.M_flags = ContactEnabledFlag

	contact.M_fixtureA = fA
	contact.M_fixtureB = fB

	contact.M_indexA = indexA
	contact.M_indexB = indexB

	contact.M_manifold = NewManifold()
	contact.M_manifold.PointCount = 0

	contact.M_prev = nil
	contact.M_next = nil

	contact.M_nodeA = NewContactEdge()

	contact.M_nodeA.Contact = nil
	contact.M_nodeA.Prev = nil
	contact.M_nodeA.Next = nil
	contact.M_nodeA.Other = nil

	contact.M_nodeB = NewContactEdge()

	contact.M_nodeB.Contact = nil
	contact.M_nodeB.Prev = nil
	contact.M_nodeB.Next = nil
	contact.M_nodeB.Other = nil

	contact.M_toiCount = 0

	contact.M_friction = MixFriction(contact.M_fixtureA.friction, contact.M_fixtureB.friction)
	contact.M_restitution = MixRestitution(contact.M_fixtureA.restitution, contact.M_fixtureB.restitution)
	contact.M_restitutionThreshold = MixRestitutionThreshold(contact.M_fixtureA.restitutionThreshold, contact.M_fixtureB.restitutionThreshold)

	contact.M_tangentSpeed = 0.0

	return contact
}

// Update the contact manifold and touching status.
// Note: do not assume the fixture AABBs are overlapping or are valid.
func ContactUpdate(contact IContact, listener IContactListener) {
	oldManifold := *contact.GetManifold()

	// Re-enable this contact.
	contact.SetFlags(contact.GetFlags() | ContactEnabledFlag)

	touching := false
	wasTouching := (contact.GetFlags() & ContactTouchingFlag) == ContactTouchingFlag

	sensorA := contact.GetFixtureA().IsSensor()
	sensorB := contact.GetFixtureB().IsSensor()
	sensor := sensorA || sensorB

	bodyA := contact.GetFixtureA().Body()
	bodyB := contact.GetFixtureB().Body()
	xfA := bodyA.Transform()
	xfB := bodyB.Transform()

	// Is this contact a sensor?
	if sensor {
		shapeA := contact.GetFixtureA().Shape()
		shapeB := contact.GetFixtureB().Shape()
		touching = TestOverlapShapes(shapeA, contact.GetChildIndexA(), shapeB, contact.GetChildIndexB(), xfA, xfB)

		// Sensors don't generate manifolds.
		contact.GetManifold().PointCount = 0
	} else {
		// *Contact is extended by specialized contact structs and mentionned by ContactInterface but not implemented on specialized structs
		// Thus when
		//spew.Dump("AVANT", contact.GetManifold())
		contact.Evaluate(contact.GetManifold(), xfA, xfB) // should be evaluated on specialisations of contact (like CircleContact)
		//spew.Dump("APRES", contact.GetManifold())
		touching = contact.GetManifold().PointCount > 0

		// Match old contact ids to new contact ids and copy the
		// stored impulses to warm start the solver.
		for i := 0; i < contact.GetManifold().PointCount; i++ {
			mp2 := &contact.GetManifold().Points[i]
			mp2.NormalImpulse = 0.0
			mp2.TangentImpulse = 0.0
			id2 := mp2.Id

			for j := 0; j < oldManifold.PointCount; j++ {
				mp1 := &oldManifold.Points[j]

				if mp1.Id.Key() == id2.Key() {
					mp2.NormalImpulse = mp1.NormalImpulse
					mp2.TangentImpulse = mp1.TangentImpulse
					break
				}
			}
		}

		if touching != wasTouching {
			bodyA.SetAwake(true)
			bodyB.SetAwake(true)
		}
	}

	if touching {
		contact.SetFlags(contact.GetFlags() | ContactTouchingFlag)
	} else {
		contact.SetFlags(contact.GetFlags() & ^ContactTouchingFlag)
	}

	if wasTouching == false && touching == true && listener != nil {
		listener.BeginContact(contact)
	}

	if wasTouching == true && touching == false && listener != nil {
		listener.EndContact(contact)
	}

	if sensor == false && touching && listener != nil {
		listener.PreSolve(contact, oldManifold)
	}
}
