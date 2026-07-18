package b2

import (
	"fmt"
)

// This holds contact filtering data.
type Filter struct {
	// The collision category bits. Normally you would just set one bit.
	CategoryBits uint16

	// The collision mask bits. This states the categories that this
	// shape would accept for collision.
	MaskBits uint16

	// Collision groups allow a certain group of objects to never collide (negative)
	// or always collide (positive). Zero means no collision group. Non-zero group
	// filtering always wins against the mask bits.
	GroupIndex int16
}

func DefaultFilter() Filter {
	return Filter{
		CategoryBits: 0x0001,
		MaskBits:     0xFFFF,
		GroupIndex:   0,
	}
}

// A fixture definition is used to create a fixture. This class defines an
// abstract fixture definition. You can reuse fixture definitions safely.
type FixtureDef struct {

	// The shape, this must be set. The shape will be cloned, so you
	// can create the shape on the stack.
	Shape IShape

	// Use this to store application specific fixture data.
	UserData any

	// The friction coefficient, usually in the range [0,1].
	Friction float64

	// The restitution (elasticity) usually in the range [0,1].
	Restitution float64

	// Restitution velocity threshold, usually in m/s. Collisions above this
	// speed have restitution applied (will bounce).
	RestitutionThreshold float64

	// The density, usually in kg/m^2.
	Density float64

	// A sensor shape collects contact information but never generates a collision
	// response.
	IsSensor bool

	// Contact filtering data.
	Filter Filter
}

// The constructor sets the default fixture definition values.
func DefaultFixtureDef() FixtureDef {
	return FixtureDef{
		Friction:             0.2,
		RestitutionThreshold: 1.0 * LengthUnitsPerMeter,
		Filter:               Filter{CategoryBits: 0x0001, MaskBits: 0xFFFF},
	}
}

// This proxy is used internally to connect fixtures to the broad-phase.
type FixtureProxy struct {
	Aabb       AABB
	Fixture    *Fixture
	ChildIndex int
	ProxyId    int
}

// A fixture is used to attach a shape to a body for collision detection. A fixture
// inherits its transform from its parent. Fixtures hold additional non-geometric data
// such as friction, collision filters, etc.
// Fixtures are created via b2Body::CreateFixture.
// @warning you cannot reuse fixtures.
type Fixture struct {
	next                 *Fixture
	body                 *Body
	shape                IShape
	density              float64
	friction             float64
	restitution          float64
	restitutionThreshold float64
	proxies              []FixtureProxy
	proxyCount           int
	filter               Filter
	isSensor             bool
	userData             any
}

func NewFixture() *Fixture {
	return &Fixture{
		filter: DefaultFilter(),
	}
}

func (fix Fixture) Type() ShapeType {
	return fix.shape.GetType()
}

func (fix Fixture) Shape() IShape {
	return fix.shape
}

func (fix Fixture) IsSensor() bool {
	return fix.isSensor
}

func (fix Fixture) FilterData() Filter {
	return fix.filter
}

func (fix Fixture) UserData() any {
	return fix.userData
}

func (fix *Fixture) SetUserData(data any) {
	fix.userData = data
}

func (fix Fixture) Body() *Body {
	return fix.body
}

func (fix Fixture) Next() *Fixture {
	return fix.next
}

// Set the density of this fixture. This will _not_ automatically adjust the mass
// of the body. You must call b2Body::ResetMassData to update the body's mass.
func (fix *Fixture) SetDensity(density float64) {
	assert(IsValid(density) && density >= 0.0)
	fix.density = density
}

// Get the density of this fixture.
func (fix Fixture) Density() float64 {
	return fix.density
}

// Get the coefficient of friction.
func (fix Fixture) Friction() float64 {
	return fix.friction
}

// Set the coefficient of friction. This will _not_ change the friction of
// existing contacts.
func (fix *Fixture) SetFriction(friction float64) {
	fix.friction = friction
}

// Get the coefficient of restitution.
func (fix Fixture) Restitution() float64 {
	return fix.restitution
}

// Set the coefficient of restitution. This will _not_ change the restitution of
// existing contacts.
func (fix *Fixture) SetRestitution(restitution float64) {
	fix.restitution = restitution
}

// Get the restitution velocity threshold.
func (fix Fixture) RestitutionThreshold() float64 {
	return fix.restitutionThreshold
}

// Set the restitution threshold. This will _not_ change the restitution threshold of
// existing contacts.
func (fix *Fixture) SetRestitutionThreshold(threshold float64) {
	fix.restitutionThreshold = threshold
}

func (fix Fixture) TestPoint(p Vec2) bool {
	return fix.shape.TestPoint(fix.body.Transform(), p)
}

func (fix Fixture) RayCast(output *RayCastOutput, input RayCastInput, childIndex int) bool {
	return fix.shape.RayCast(output, input, fix.body.Transform(), childIndex)
}

func (fix Fixture) MassData(massData *MassData) {
	fix.shape.ComputeMass(massData, fix.density)
}

// Get the fixture's AABB. This AABB may be enlarge and/or stale.
// If you need a more accurate AABB, compute it using the shape and
// the body transform.
func (fix Fixture) AABB(childIndex int) AABB {
	assert(0 <= childIndex && childIndex < fix.proxyCount)
	return fix.proxies[childIndex].Aabb
}

func (fix *Fixture) Create(body *Body, def *FixtureDef) {
	fix.userData = def.UserData
	fix.friction = def.Friction
	fix.restitution = def.Restitution
	fix.restitutionThreshold = def.RestitutionThreshold

	fix.body = body
	fix.next = nil

	fix.filter = def.Filter

	fix.isSensor = def.IsSensor

	fix.shape = def.Shape.Clone()

	// Reserve proxy space
	childCount := fix.shape.GetChildCount()
	fix.proxies = make([]FixtureProxy, childCount)

	for i := range childCount {
		fix.proxies[i].Fixture = nil
		fix.proxies[i].ProxyId = E_nullProxy
	}
	fix.proxyCount = 0

	fix.density = def.Density
}

func (fix *Fixture) Destroy() {

	// The proxies must be destroyed before calling this.
	assert(fix.proxyCount == 0)

	// Free the proxy array.
	fix.proxies = nil

	// Free the child shape.
	switch fix.shape.GetType() {
	case Circle:
		s := fix.shape.(*CircleShape)
		s.Destroy()
	case Edge:
		s := fix.shape.(*EdgeShape)
		s.Destroy()
	case Polygon:
		s := fix.shape.(*PolygonShape)
		s.Destroy()
	case Chain:
		s := fix.shape.(*ChainShape)
		s.Destroy()
	default:
		assert(false)
	}

	fix.shape = nil
}

func (fix *Fixture) CreateProxies(broadPhase *BroadPhase, xf Transform) {
	assert(fix.proxyCount == 0)

	// Create proxies in the broad-phase.
	fix.proxyCount = fix.shape.GetChildCount()

	for i := 0; i < fix.proxyCount; i++ {
		proxy := &fix.proxies[i]
		fix.shape.ComputeAABB(&proxy.Aabb, xf, i)
		proxy.ProxyId = broadPhase.CreateProxy(proxy.Aabb, proxy)
		proxy.Fixture = fix
		proxy.ChildIndex = i
	}
}

func (fix *Fixture) DestroyProxies(broadPhase *BroadPhase) {
	// Destroy proxies in the broad-phase.
	for i := 0; i < fix.proxyCount; i++ {
		proxy := &fix.proxies[i]
		broadPhase.DestroyProxy(proxy.ProxyId)
		proxy.ProxyId = E_nullProxy
	}

	fix.proxyCount = 0
}

func (fix *Fixture) Synchronize(broadPhase *BroadPhase, transform1 Transform, transform2 Transform) {

	if fix.proxyCount == 0 {
		return
	}

	for i := 0; i < fix.proxyCount; i++ {

		proxy := &fix.proxies[i]

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		aabb1 := AABB{}
		aabb2 := AABB{}
		fix.shape.ComputeAABB(&aabb1, transform1, proxy.ChildIndex)
		fix.shape.ComputeAABB(&aabb2, transform2, proxy.ChildIndex)

		proxy.Aabb.CombineTwoInPlace(aabb1, aabb2)

		displacement := Vec2Sub(aabb2.GetCenter(), aabb1.GetCenter())

		broadPhase.MoveProxy(proxy.ProxyId, proxy.Aabb, displacement)
	}
}

func (fix *Fixture) SetFilterData(filter Filter) {
	fix.filter = filter
	fix.Refilter()
}

func (fix *Fixture) Refilter() {

	if fix.body == nil {
		return
	}

	// Flag associated contacts for filtering.
	edge := fix.body.ContactList()
	for edge != nil {
		contact := edge.Contact
		fixtureA := contact.GetFixtureA()
		fixtureB := contact.GetFixtureB()
		if fixtureA == fix || fixtureB == fix {
			contact.FlagForFiltering()
		}

		edge = edge.Next
	}

	world := fix.body.GetWorld()

	if world == nil {
		return
	}

	// Touch each proxy so that new pairs may be created
	broadPhase := &world.contactManager.broadPhase
	for i := 0; i < fix.proxyCount; i++ {
		broadPhase.TouchProxy(fix.proxies[i].ProxyId)
	}
}

func (fix *Fixture) SetSensor(sensor bool) {
	if sensor != fix.isSensor {
		fix.body.SetAwake(true)
		fix.isSensor = sensor
	}
}

func (fix *Fixture) Dump(bodyIndex int) {
	fmt.Print(fmt.Printf("    b2FixtureDef fd;\n"))
	fmt.Print(fmt.Printf("    fd.friction = %.15f;\n", fix.friction))
	fmt.Print(fmt.Printf("    fd.restitution = %.15f;\n", fix.restitution))
	fmt.Print(fmt.Printf("    fd.restitutionThreshold = %.15f;\n", fix.restitutionThreshold))
	fmt.Print(fmt.Printf("    fd.density = %.15f;\n", fix.density))
	fmt.Print(fmt.Printf("    fd.isSensor = bool(%v);\n", fix.isSensor))
	fmt.Print(fmt.Printf("    fd.filter.categoryBits = uint16(%d);\n", fix.filter.CategoryBits))
	fmt.Print(fmt.Printf("    fd.filter.maskBits = uint16(%d);\n", fix.filter.MaskBits))
	fmt.Print(fmt.Printf("    fd.filter.groupIndex = int16(%d);\n", fix.filter.GroupIndex))

	switch fix.shape.GetType() {
	case Circle:
		{
			s := fix.shape.(*CircleShape)
			fmt.Print(fmt.Printf("    b2CircleShape shape;\n"))
			fmt.Print(fmt.Printf("    shape.m_radius = %.15f;\n", s.radius))
			fmt.Print(fmt.Printf("    shape.m_p.Set(%.15f, %.15f);\n", s.pos.X, s.pos.Y))
		}

	case Edge:
		{
			s := fix.shape.(*EdgeShape)
			fmt.Print(fmt.Printf("    b2EdgeShape shape;\n"))
			fmt.Print(fmt.Printf("    shape.m_radius = %.15f;\n", s.radius))
			fmt.Print(fmt.Printf("    shape.m_vertex0.Set(%.15f, %.15f);\n", s.M_vertex0.X, s.M_vertex0.Y))
			fmt.Print(fmt.Printf("    shape.m_vertex1.Set(%.15f, %.15f);\n", s.M_vertex1.X, s.M_vertex1.Y))
			fmt.Print(fmt.Printf("    shape.m_vertex2.Set(%.15f, %.15f);\n", s.M_vertex2.X, s.M_vertex2.Y))
			fmt.Print(fmt.Printf("    shape.m_vertex3.Set(%.15f, %.15f);\n", s.M_vertex3.X, s.M_vertex3.Y))
			fmt.Print(fmt.Printf("    shape.m_oneSided = bool(%v);\\n", s.M_oneSided))
		}

	case Polygon:
		{
			s := fix.shape.(*PolygonShape)
			fmt.Print(fmt.Printf("    b2PolygonShape shape;\n"))
			fmt.Print(fmt.Printf("    b2Vec2 vs[%d];\n", MaxPolygonVertices))
			for i := 0; i < s.Count; i++ {
				fmt.Print(fmt.Printf("    vs[%d].Set(%.15f, %.15f);\n", i, s.Vertices[i].X, s.Vertices[i].Y))
			}
			fmt.Print(fmt.Printf("    shape.Set(vs, %d);\n", s.Count))
		}

	case Chain:
		{
			s := fix.shape.(*ChainShape)
			fmt.Print(fmt.Printf("    b2ChainShape shape;\n"))
			fmt.Print(fmt.Printf("    b2Vec2 vs[%d];\n", s.M_count))
			for i := 0; i < s.M_count; i++ {
				fmt.Print(fmt.Printf("    vs[%d].Set(%.15f, %.15f);\n", i, s.M_vertices[i].X, s.M_vertices[i].Y))
			}
			fmt.Print(fmt.Printf("    shape.CreateChain(vs, %d);\n", s.M_count))
			fmt.Print(fmt.Printf("    shape.m_prevVertex.Set(%.15f, %.15f);\n", s.M_prevVertex.X, s.M_prevVertex.Y))
			fmt.Print(fmt.Printf("    shape.m_nextVertex.Set(%.15f, %.15f);\n", s.M_nextVertex.X, s.M_nextVertex.Y))
		}

	default:
		return
	}

	fmt.Print("\n")
	fmt.Print("    fd.shape = &shape;\n")
	fmt.Print("\n")
	fmt.Print(fmt.Printf("    bodies[%d].CreateFixture(&fd);\n", bodyIndex))
}
