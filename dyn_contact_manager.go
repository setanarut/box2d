package b2

type ContactManager struct {
	broadPhase      BroadPhase
	contactList     IContact
	contactCount    int
	contactFilter   IContactFilter
	contactListener IContactListener
}

func MakeContactManager() ContactManager {
	return ContactManager{
		broadPhase:    MakeBroadPhase(),
		contactFilter: &ContactFilter{},
	}
}

func NewContactManager() *ContactManager {
	res := MakeContactManager()
	return &res
}

func (mgr *ContactManager) Destroy(c IContact) {
	fixtureA := c.GetFixtureA()
	fixtureB := c.GetFixtureB()
	bodyA := fixtureA.Body()
	bodyB := fixtureB.Body()

	if mgr.contactListener != nil && c.IsTouching() {
		mgr.contactListener.EndContact(c)
	}

	// Remove from the world.
	if c.GetPrev() != nil {
		c.GetPrev().SetNext(c.GetNext())
	}

	if c.GetNext() != nil {
		c.GetNext().SetPrev(c.GetPrev())
	}

	if c == mgr.contactList {
		mgr.contactList = c.GetNext()
	}

	// Remove from body 1
	if c.GetNodeA().Prev != nil {
		c.GetNodeA().Prev.Next = c.GetNodeA().Next
	}

	if c.GetNodeA().Next != nil {
		c.GetNodeA().Next.Prev = c.GetNodeA().Prev
	}

	if c.GetNodeA() == bodyA.contactList {
		bodyA.contactList = c.GetNodeA().Next
	}

	// Remove from body 2
	if c.GetNodeB().Prev != nil {
		c.GetNodeB().Prev.Next = c.GetNodeB().Next
	}

	if c.GetNodeB().Next != nil {
		c.GetNodeB().Next.Prev = c.GetNodeB().Prev
	}

	if c.GetNodeB() == bodyB.contactList {
		bodyB.contactList = c.GetNodeB().Next
	}

	// Call the factory.
	ContactDestroy(c)
	mgr.contactCount--
}

// This is the top level collision call for the time step. Here
// all the narrow phase collision is processed for the world
// contact list.
func (mgr *ContactManager) Collide() {
	// Update awake contacts.
	c := mgr.contactList

	for c != nil {
		fixtureA := c.GetFixtureA()
		fixtureB := c.GetFixtureB()
		indexA := c.GetChildIndexA()
		indexB := c.GetChildIndexB()
		bodyA := fixtureA.Body()
		bodyB := fixtureB.Body()

		// Is this contact flagged for filtering?
		if (c.GetFlags() & ContactFilterFlag) != 0x0000 {
			// Should these bodies collide?
			if bodyB.ShouldCollide(bodyA) == false {
				cNuke := c
				c = cNuke.GetNext()
				mgr.Destroy(cNuke)
				continue
			}

			// Check user filtering.
			if mgr.contactFilter != nil && mgr.contactFilter.ShouldCollide(fixtureA, fixtureB) == false {
				cNuke := c
				c = cNuke.GetNext()
				mgr.Destroy(cNuke)
				continue
			}

			// Clear the filtering flag.
			c.SetFlags(c.GetFlags() & ^ContactFilterFlag)
		}

		activeA := bodyA.IsAwake() && bodyA.bodyType != Static
		activeB := bodyB.IsAwake() && bodyB.bodyType != Static

		// At least one body must be awake and it must be dynamic or kinematic.
		if activeA == false && activeB == false {
			c = c.GetNext()
			continue
		}

		proxyIdA := fixtureA.proxies[indexA].ProxyId
		proxyIdB := fixtureB.proxies[indexB].ProxyId
		overlap := mgr.broadPhase.TestOverlap(proxyIdA, proxyIdB)

		// Here we destroy contacts that cease to overlap in the broad-phase.
		if overlap == false {
			cNuke := c
			c = cNuke.GetNext()
			mgr.Destroy(cNuke)
			continue
		}

		// The contact persists.
		ContactUpdate(c, mgr.contactListener)
		c = c.GetNext()
	}
}

func (mgr *ContactManager) FindNewContacts() {
	mgr.broadPhase.UpdatePairs(mgr.AddPair)
}

func (mgr *ContactManager) AddPair(proxyUserDataA any, proxyUserDataB any) {

	proxyA := proxyUserDataA.(*FixtureProxy)
	proxyB := proxyUserDataB.(*FixtureProxy)

	fixtureA := proxyA.Fixture
	fixtureB := proxyB.Fixture

	indexA := proxyA.ChildIndex
	indexB := proxyB.ChildIndex

	bodyA := fixtureA.Body()
	bodyB := fixtureB.Body()

	// Are the fixtures on the same body?
	if bodyA == bodyB {
		return
	}

	// TODO_ERIN use a hash table to remove a potential bottleneck when both
	// bodies have a lot of contacts.
	// Does a contact already exist?
	edge := bodyB.ContactList()
	for edge != nil {
		if edge.Other == bodyA {
			fA := edge.Contact.GetFixtureA()
			fB := edge.Contact.GetFixtureB()
			iA := edge.Contact.GetChildIndexA()
			iB := edge.Contact.GetChildIndexB()

			if fA == fixtureA && fB == fixtureB && iA == indexA && iB == indexB {
				// A contact already exists.
				return
			}

			if fA == fixtureB && fB == fixtureA && iA == indexB && iB == indexA {
				// A contact already exists.
				return
			}
		}

		edge = edge.Next
	}

	// Does a joint override collision? Is at least one body dynamic?
	if bodyB.ShouldCollide(bodyA) == false {
		return
	}

	// Check user filtering.
	if mgr.contactFilter != nil && mgr.contactFilter.ShouldCollide(fixtureA, fixtureB) == false {
		return
	}

	// Call the factory.
	c := ContactFactory(fixtureA, indexA, fixtureB, indexB)
	if c == nil {
		return
	}

	// Contact creation may swap fixtures.
	fixtureA = c.GetFixtureA()
	fixtureB = c.GetFixtureB()
	indexA = c.GetChildIndexA()
	indexB = c.GetChildIndexB()
	bodyA = fixtureA.Body()
	bodyB = fixtureB.Body()

	// Insert into the world.
	c.SetPrev(nil)
	c.SetNext(mgr.contactList)
	if mgr.contactList != nil {
		mgr.contactList.SetPrev(c)
	}
	mgr.contactList = c

	// Connect to island graph.

	// Connect to body A
	// fmt.Printf("getNode(): %p\n", c.GetNodeA())
	// fmt.Printf("getNode(): %p\n", c.GetNodeA())
	// fmt.Printf("getNode(): %p\n", c.GetNodeA())

	c.GetNodeA().Contact = c
	c.GetNodeA().Other = bodyB

	c.GetNodeA().Prev = nil
	c.GetNodeA().Next = bodyA.contactList
	if bodyA.contactList != nil {
		bodyA.contactList.Prev = c.GetNodeA()
	}
	bodyA.contactList = c.GetNodeA()

	// Connect to body B
	c.GetNodeB().Contact = c
	c.GetNodeB().Other = bodyA

	c.GetNodeB().Prev = nil
	c.GetNodeB().Next = bodyB.contactList
	if bodyB.contactList != nil {
		bodyB.contactList.Prev = c.GetNodeB()
	}
	bodyB.contactList = c.GetNodeB()

	mgr.contactCount++
}
