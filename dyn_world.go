package b2

import (
	"fmt"
	"math"
)

// World struct manages all physics entities, dynamic simulation,
// and asynchronous queries. The world also contains efficient memory
// management facilities.
type World struct {
	contactManager      ContactManager
	bodyList            *Body  // linked list
	jointList           IJoint // has to be backed by pointer
	bodyCount           int
	jointCount          int
	gravity             Vec2
	allowSleep          bool
	destructionListener IDestructionListener
	// This is used to compute the time step ratio to
	// support a variable time step.
	inv_dt0     float64
	newContacts bool
	locked      bool
	clearForces bool
	// These are for debugging the solver.
	warmStarting      bool
	continuousPhysics bool
	subStepping       bool
	stepComplete      bool
	profile           Profile
}

func (world World) GetBodyList() *Body {
	return world.bodyList
}

func (world World) GetJointList() IJoint { // returns a pointer
	return world.jointList
}

// Get the world contact list. With the returned contact,
// use Contact.GetNext() to get the next contact in the world list.
// A nullptr contact indicates the end of the list.
func (world World) GetContactList() IContact {
	return world.contactManager.contactList
}

func (world World) GetBodyCount() int {
	return world.bodyCount
}

func (world World) GetJointCount() int {
	return world.jointCount
}

func (world World) GetContactCount() int {
	return world.contactManager.contactCount
}

func (world *World) SetGravity(gravity Vec2) {
	world.gravity = gravity
}

func (world World) GetGravity() Vec2 {
	return world.gravity
}

func (world World) IsLocked() bool {
	return world.locked
}

func (world *World) SetAutoClearForces(flag bool) {
	world.clearForces = flag
}

// Get the flag that controls automatic clearing of forces after each time step.
func (world World) GetAutoClearForces() bool {
	return world.clearForces
}

func (world World) GetContactManager() ContactManager {
	return world.contactManager
}

func (world World) GetProfile() Profile {
	return world.profile
}

func MakeWorld(gravity Vec2) World {
	return World{
		contactManager:    MakeContactManager(),
		gravity:           gravity,
		allowSleep:        true,
		clearForces:       true,
		warmStarting:      true,
		continuousPhysics: true,
		stepComplete:      true,
	}
}

func (world *World) Destroy() {

	// Some shapes allocate using b2Alloc.
	b := world.bodyList
	for b != nil {
		bNext := b.next

		f := b.fixtureList
		for f != nil {
			fNext := f.next
			f.proxyCount = 0
			f.Destroy()
			f = fNext
		}

		b = bNext
	}
}

func (world *World) SetDestructionListener(listener IDestructionListener) {
	world.destructionListener = listener
}

func (world *World) SetContactFilter(filter IContactFilter) {
	world.contactManager.contactFilter = filter
}

func (world *World) SetContactListener(listener IContactListener) {
	world.contactManager.contactListener = listener
}

func (world *World) CreateBody(def *BodyDef) *Body {
	assert(world.IsLocked() == false)

	if world.IsLocked() {
		return nil
	}

	b := NewBody(def, world)

	// Add to world doubly linked list.
	b.prev = nil
	b.next = world.bodyList
	if world.bodyList != nil {
		world.bodyList.prev = b
	}
	world.bodyList = b
	world.bodyCount++

	return b
}

func (world *World) DestroyBody(b *Body) {
	assert(world.bodyCount > 0)
	assert(world.IsLocked() == false)

	if world.IsLocked() {
		return
	}

	// Delete the attached joints.
	je := b.jointList
	for je != nil {
		je0 := je
		je = je.Next

		if world.destructionListener != nil {
			world.destructionListener.SayGoodbyeToJoint(je0.Joint)
		}

		world.DestroyJoint(je0.Joint)

		b.jointList = je
	}
	b.jointList = nil

	// Delete the attached contacts.
	ce := b.contactList
	for ce != nil {
		ce0 := ce
		ce = ce.Next
		world.contactManager.Destroy(ce0.Contact)
	}
	b.contactList = nil

	// Delete the attached fixtures. This destroys broad-phase proxies.
	f := b.fixtureList
	for f != nil {
		f0 := f
		f = f.next

		if world.destructionListener != nil {
			world.destructionListener.SayGoodbyeToFixture(f0)
		}

		f0.DestroyProxies(&world.contactManager.broadPhase)
		f0.Destroy()

		b.fixtureList = f
		b.fixtureCount -= 1
	}

	b.fixtureList = nil
	b.fixtureCount = 0

	// Remove world body list.
	if b.prev != nil {
		b.prev.next = b.next
	}

	if b.next != nil {
		b.next.prev = b.prev
	}

	if b == world.bodyList {
		world.bodyList = b.next
	}

	world.bodyCount--
}

func (world *World) CreateJoint(def IJointDef) IJoint {
	assert(world.IsLocked() == false)
	if world.IsLocked() {
		return nil
	}

	j := JointCreate(def)

	// Connect to the world list.
	j.SetPrev(nil)
	j.SetNext(world.jointList)
	if world.jointList != nil {
		world.jointList.SetPrev(j)
	}
	world.jointList = j
	world.jointCount++

	// Connect to the bodies' doubly linked lists.
	j.GetEdgeA().Joint = j
	j.GetEdgeA().Other = j.GetBodyB()
	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = j.GetBodyA().jointList
	if j.GetBodyA().jointList != nil {
		j.GetBodyA().jointList.Prev = j.GetEdgeA()
	}

	j.GetBodyA().jointList = j.GetEdgeA()

	j.GetEdgeB().Joint = j
	j.GetEdgeB().Other = j.GetBodyA()
	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = j.GetBodyB().jointList
	if j.GetBodyB().jointList != nil {
		j.GetBodyB().jointList.Prev = j.GetEdgeB()
	}
	j.GetBodyB().jointList = j.GetEdgeB()

	bodyA := def.GetBodyA()
	bodyB := def.GetBodyB()

	// If the joint prevents collisions, then flag any contacts for filtering.
	if def.IsCollideConnected() == false {
		edge := bodyB.ContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}

	// Note: creating a joint doesn't wake the bodies.

	return j
}

func (world *World) DestroyJoint(j IJoint) { // j backed by pointer
	assert(world.IsLocked() == false)
	if world.IsLocked() {
		return
	}

	collideConnected := j.IsCollideConnected()

	// Remove from the doubly linked list.
	if j.GetPrev() != nil {
		j.GetPrev().SetNext(j.GetNext())
	}

	if j.GetNext() != nil {
		j.GetNext().SetPrev(j.GetPrev())
	}

	if j == world.jointList {
		world.jointList = j.GetNext()
	}

	// Disconnect from island graph.
	bodyA := j.GetBodyA()
	bodyB := j.GetBodyB()

	// Wake up connected bodies.
	bodyA.SetAwake(true)
	bodyB.SetAwake(true)

	// Remove from body 1.
	if j.GetEdgeA().Prev != nil {
		j.GetEdgeA().Prev.Next = j.GetEdgeA().Next
	}

	if j.GetEdgeA().Next != nil {
		j.GetEdgeA().Next.Prev = j.GetEdgeA().Prev
	}

	if j.GetEdgeA() == bodyA.jointList {
		bodyA.jointList = j.GetEdgeA().Next
	}

	j.GetEdgeA().Prev = nil
	j.GetEdgeA().Next = nil

	// Remove from body 2
	if j.GetEdgeB().Prev != nil {
		j.GetEdgeB().Prev.Next = j.GetEdgeB().Next
	}

	if j.GetEdgeB().Next != nil {
		j.GetEdgeB().Next.Prev = j.GetEdgeB().Prev
	}

	if j.GetEdgeB() == bodyB.jointList {
		bodyB.jointList = j.GetEdgeB().Next
	}

	j.GetEdgeB().Prev = nil
	j.GetEdgeB().Next = nil

	JointDestroy(j)

	assert(world.jointCount > 0)
	world.jointCount--

	// If the joint prevents collisions, then flag any contacts for filtering.
	if collideConnected == false {
		edge := bodyB.ContactList()
		for edge != nil {
			if edge.Other == bodyA {
				// Flag the contact for filtering at the next time step (where either
				// body is awake).
				edge.Contact.FlagForFiltering()
			}

			edge = edge.Next
		}
	}
}

func (world *World) SetAllowSleeping(flag bool) {
	if flag == world.allowSleep {
		return
	}

	world.allowSleep = flag
	if world.allowSleep == false {
		for b := world.bodyList; b != nil; b = b.next {
			b.SetAwake(true)
		}
	}
}

// Find islands, integrate and solve constraints, solve position constraints
func (world *World) Solve(step TimeStep) {
	world.profile.SolveInit = 0.0
	world.profile.SolveVelocity = 0.0
	world.profile.SolvePosition = 0.0

	// Size the island for the worst case.
	island := MakeIsland(
		world.bodyCount,
		world.contactManager.contactCount,
		world.jointCount,
		world.contactManager.contactListener,
	)

	// Clear all the island flags.
	for b := world.bodyList; b != nil; b = b.next {
		b.flags &= ^BodyIslandFlag
	}
	for c := world.contactManager.contactList; c != nil; c = c.GetNext() {
		c.SetFlags(c.GetFlags() & ^BodyIslandFlag)
	}

	for j := world.jointList; j != nil; j = j.GetNext() {
		j.SetIslandFlag(false)
	}

	// Build and simulate all awake islands.
	stackSize := world.bodyCount
	stack := make([]*Body, stackSize)

	for seed := world.bodyList; seed != nil; seed = seed.next {
		if (seed.flags & BodyIslandFlag) != 0x0000 {
			continue
		}

		if seed.IsAwake() == false || seed.IsEnabled() == false {
			continue
		}

		// The seed can be dynamic or kinematic.
		if seed.bodyType == Static {
			continue
		}

		// Reset island and stack.
		island.Clear()
		stackCount := 0
		stack[stackCount] = seed
		stackCount++
		seed.flags |= BodyIslandFlag

		// Perform a depth first search (DFS) on the constraint graph.
		for stackCount > 0 {
			// Grab the next body off the stack and add it to the island.
			stackCount--
			b := stack[stackCount]
			assert(b.IsEnabled() == true)
			island.AddBody(b)

			// To keep islands as small as possible, we don't
			// propagate islands across static bodies.
			if b.Type() == Static {
				continue
			}

			// Make sure the body is awake (without resetting sleep timer).
			b.flags |= BodyAwakeFlag

			// Search all contacts connected to this body.
			for ce := b.contactList; ce != nil; ce = ce.Next {
				contact := ce.Contact

				// Has this contact already been added to an island?
				if (contact.GetFlags() & BodyIslandFlag) != 0x0000 {
					continue
				}

				// Is this contact solid and touching?
				if contact.IsEnabled() == false || contact.IsTouching() == false {
					continue
				}

				// Skip sensors.
				sensorA := contact.GetFixtureA().isSensor
				sensorB := contact.GetFixtureB().isSensor

				if sensorA || sensorB {
					continue
				}

				island.AddContact(contact)
				contact.SetFlags(contact.GetFlags() | BodyIslandFlag)

				other := ce.Other

				// Was the other body already added to this island?
				if (other.flags & BodyIslandFlag) != 0x0000 {
					continue
				}

				assert(stackCount < stackSize)
				stack[stackCount] = other
				stackCount++
				other.flags |= BodyIslandFlag
			}

			// Search all joints connect to this body.
			for je := b.jointList; je != nil; je = je.Next {

				if je.Joint.GetIslandFlag() == true {
					continue
				}

				other := je.Other

				// Don't simulate joints connected to disabled bodies.
				if other.IsEnabled() == false {
					continue
				}

				island.Add(je.Joint)
				je.Joint.SetIslandFlag(true)

				if other.flags&BodyIslandFlag != 0x0000 {
					continue
				}

				assert(stackCount < stackSize)
				stack[stackCount] = other
				stackCount++
				other.flags |= BodyIslandFlag
			}
		}

		profile := Profile{}
		island.Solve(&profile, step, world.gravity, world.allowSleep)
		world.profile.SolveInit += profile.SolveInit
		world.profile.SolveVelocity += profile.SolveVelocity
		world.profile.SolvePosition += profile.SolvePosition

		// Post solve cleanup.
		for i := 0; i < island.M_bodyCount; i++ {
			// Allow static bodies to participate in other islands.
			b := island.M_bodies[i]
			if b.Type() == Static {
				b.flags &= ^BodyIslandFlag
			}
		}
	}

	stack = nil

	{
		timer := MakeTimer()

		// Synchronize fixtures, check for out of range bodies.
		for b := world.bodyList; b != nil; b = b.Next() {
			// If a body was not in an island then it did not move.
			if (b.flags & BodyIslandFlag) == 0 {
				continue
			}

			if b.Type() == Static {
				continue
			}

			// Update fixtures (for broad-phase).
			b.SynchronizeFixtures()
		}

		// Look for new contacts.
		world.contactManager.FindNewContacts()
		world.profile.Broadphase = timer.GetMilliseconds()
	}
}

// Find TOI contacts and solve them.
func (world *World) SolveTOI(step TimeStep) {

	island := MakeIsland(2*maxTOIContacts, maxTOIContacts, 0, world.contactManager.contactListener)

	if world.stepComplete {
		for b := world.bodyList; b != nil; b = b.next {
			b.flags &= ^BodyIslandFlag
			b.sweep.Alpha0 = 0.0
		}

		for c := world.contactManager.contactList; c != nil; c = c.GetNext() {
			// Invalidate TOI
			c.SetFlags(c.GetFlags() & ^(BodyToiFlag | BodyIslandFlag))
			c.SetTOICount(0)
			c.SetTOI(1.0)
		}
	}

	// Find TOI events and solve them.
	for {
		// Find the first TOI.
		var minContact IContact = nil // has to be a pointer
		minAlpha := 1.0

		for c := world.contactManager.contactList; c != nil; c = c.GetNext() {

			// Is this contact disabled?
			if c.IsEnabled() == false {
				continue
			}

			// Prevent excessive sub-stepping.
			if c.GetTOICount() > maxSubSteps {
				continue
			}

			alpha := 1.0
			if (c.GetFlags() & BodyToiFlag) != 0x0000 {
				// This contact has a valid cached TOI.
				alpha = c.GetTOI()
			} else {
				fA := c.GetFixtureA()
				fB := c.GetFixtureB()

				// Is there a sensor?
				if fA.IsSensor() || fB.IsSensor() {
					continue
				}

				bA := fA.Body()
				bB := fB.Body()

				typeA := bA.bodyType
				typeB := bB.bodyType
				assert(typeA == Dynamic || typeB == Dynamic)

				activeA := bA.IsAwake() && typeA != Static
				activeB := bB.IsAwake() && typeB != Static

				// Is at least one body active (awake and dynamic or kinematic)?
				if activeA == false && activeB == false {
					continue
				}

				collideA := bA.IsBullet() || typeA != Dynamic
				collideB := bB.IsBullet() || typeB != Dynamic

				// Are these two non-bullet dynamic bodies?
				if collideA == false && collideB == false {
					continue
				}

				// Compute the TOI for this contact.
				// Put the sweeps onto the same time interval.
				alpha0 := bA.sweep.Alpha0

				if bA.sweep.Alpha0 < bB.sweep.Alpha0 {
					alpha0 = bB.sweep.Alpha0
					bA.sweep.Advance(alpha0)
				} else if bB.sweep.Alpha0 < bA.sweep.Alpha0 {
					alpha0 = bA.sweep.Alpha0
					bB.sweep.Advance(alpha0)
				}

				assert(alpha0 < 1.0)

				indexA := c.GetChildIndexA()
				indexB := c.GetChildIndexB()

				// Compute the time of impact in interval [0, minTOI]
				input := MakeTOIInput()
				input.ProxyA.Set(fA.Shape(), indexA)
				input.ProxyB.Set(fB.Shape(), indexB)
				input.SweepA = bA.sweep
				input.SweepB = bB.sweep
				input.TMax = 1.0

				output := MakeTOIOutput()
				TimeOfImpact(&output, &input)

				// Beta is the fraction of the remaining portion of the .
				beta := output.T
				if output.State == toiTouching {
					alpha = math.Min(alpha0+(1.0-alpha0)*beta, 1.0)
				} else {
					alpha = 1.0
				}

				c.SetTOI(alpha)
				c.SetFlags(c.GetFlags() | BodyToiFlag)
			}

			if alpha < minAlpha {
				// This is the minimum TOI found so far.
				minContact = c
				minAlpha = alpha
			}
		}

		if minContact == nil || 1.0-10.0*epsilon < minAlpha {
			// No more TOI events. Done!
			world.stepComplete = true
			break
		}

		// Advance the bodies to the TOI.
		fA := minContact.GetFixtureA()
		fB := minContact.GetFixtureB()
		bA := fA.Body()
		bB := fB.Body()

		backup1 := bA.sweep
		backup2 := bB.sweep

		bA.Advance(minAlpha)
		bB.Advance(minAlpha)

		// The TOI contact likely has some new contact points.
		ContactUpdate(minContact, world.contactManager.contactListener)
		minContact.SetFlags(minContact.GetFlags() & ^BodyToiFlag)
		minContact.SetTOICount(minContact.GetTOICount() + 1)

		// Is the contact solid?
		if minContact.IsEnabled() == false || minContact.IsTouching() == false {
			// Restore the sweeps.
			minContact.SetEnabled(false)
			bA.sweep = backup1
			bB.sweep = backup2
			bA.SynchronizeTransform()
			bB.SynchronizeTransform()
			continue
		}

		bA.SetAwake(true)
		bB.SetAwake(true)

		// Build the island
		island.Clear()
		island.AddBody(bA)
		island.AddBody(bB)
		island.AddContact(minContact)

		bA.flags |= BodyIslandFlag
		bB.flags |= BodyIslandFlag
		minContact.SetFlags(minContact.GetFlags() | BodyIslandFlag)

		// Get contacts on bodyA and bodyB.
		bodies := [2]*Body{bA, bB}

		for i := range 2 {
			body := bodies[i]
			if body.bodyType == Dynamic {
				for ce := body.contactList; ce != nil; ce = ce.Next {
					if island.M_bodyCount == island.M_bodyCapacity {
						break
					}

					if island.M_contactCount == island.M_contactCapacity {
						break
					}

					contact := ce.Contact

					// Has this contact already been added to the island?
					if (contact.GetFlags() & BodyIslandFlag) != 0x0000 {
						continue
					}

					// Only add static, kinematic, or bullet bodies.
					other := ce.Other
					if other.bodyType == Dynamic && body.IsBullet() == false && other.IsBullet() == false {
						continue
					}

					// Skip sensors.
					sensorA := contact.GetFixtureA().isSensor
					sensorB := contact.GetFixtureB().isSensor
					if sensorA || sensorB {
						continue
					}

					// Tentatively advance the body to the TOI.
					backup := other.sweep
					if (other.flags & BodyIslandFlag) == 0 {
						other.Advance(minAlpha)
					}

					// Update the contact points
					ContactUpdate(contact, world.contactManager.contactListener)

					// Was the contact disabled by the user?
					if contact.IsEnabled() == false {
						other.sweep = backup
						other.SynchronizeTransform()
						continue
					}

					// Are there contact points?
					if contact.IsTouching() == false {
						other.sweep = backup
						other.SynchronizeTransform()
						continue
					}

					// Add the contact to the island
					contact.SetFlags(contact.GetFlags() | BodyIslandFlag)
					island.AddContact(contact)

					// Has the other body already been added to the island?
					if (other.flags & BodyIslandFlag) != 0x0000 {
						continue
					}

					// Add the other body to the island.
					other.flags |= BodyIslandFlag

					if other.bodyType != Static {
						other.SetAwake(true)
					}

					island.AddBody(other)
				}
			}
		}

		subStep := MakeTimeStep()
		subStep.Dt = (1.0 - minAlpha) * step.Dt
		subStep.Inv_dt = 1.0 / subStep.Dt
		subStep.DtRatio = 1.0
		subStep.PositionIterations = 20
		subStep.VelocityIterations = step.VelocityIterations
		subStep.WarmStarting = false
		island.SolveTOI(subStep, bA.islandIndex, bB.islandIndex)

		// Reset island flags and synchronize broad-phase proxies.
		for i := 0; i < island.M_bodyCount; i++ {
			body := island.M_bodies[i]
			body.flags &= ^BodyIslandFlag

			if body.bodyType != Dynamic {
				continue
			}

			body.SynchronizeFixtures()

			// Invalidate all contact TOIs on this displaced body.
			for ce := body.contactList; ce != nil; ce = ce.Next {
				ce.Contact.SetFlags(ce.Contact.GetFlags() & ^(BodyToiFlag | BodyIslandFlag))
			}
		}

		// Commit fixture proxy movements to the broad-phase so that new contacts are created.
		// Also, some contacts can be destroyed.
		world.contactManager.FindNewContacts()

		if world.subStepping {
			world.stepComplete = false
			break
		}
	}
}

func (world *World) Step(dt float64, velocityIterations int, positionIterations int) {
	stepTimer := MakeTimer()

	// If new fixtures were added, we need to find the new contacts.
	if world.newContacts {
		world.contactManager.FindNewContacts()
		world.newContacts = false
	}

	world.locked = true

	step := MakeTimeStep()
	step.Dt = dt
	step.VelocityIterations = velocityIterations
	step.PositionIterations = positionIterations
	if dt > 0.0 {
		step.Inv_dt = 1.0 / dt
	} else {
		step.Inv_dt = 0.0
	}

	step.DtRatio = world.inv_dt0 * dt

	step.WarmStarting = world.warmStarting

	// Integrate velocities, solve velocity constraints, and integrate positions.
	if world.stepComplete && step.Dt > 0.0 {
		timer := MakeTimer()
		world.Solve(step)
		world.profile.Solve = timer.GetMilliseconds()
	}

	// Update contacts. This is where some contacts are destroyed.
	{
		timer := MakeTimer()
		world.contactManager.Collide()
		world.profile.Collide = timer.GetMilliseconds()
	}

	// Handle TOI events.
	if world.continuousPhysics && step.Dt > 0.0 {
		timer := MakeTimer()
		world.SolveTOI(step)
		world.profile.SolveTOI = timer.GetMilliseconds()
	}

	if step.Dt > 0.0 {
		world.inv_dt0 = step.Inv_dt
	}

	if world.clearForces {
		world.ClearForces()
	}

	world.locked = false

	world.profile.Step = stepTimer.GetMilliseconds()
}

func (world *World) ClearForces() {
	for body := world.bodyList; body != nil; body = body.Next() {
		body.force.SetZero()
		body.torque = 0.0
	}
}

type WorldQueryWrapper struct {
	BroadPhase *BroadPhase
	Callback   BroadPhaseQueryCallback
}

func MakeWorldQueryWrapper() WorldQueryWrapper {
	return WorldQueryWrapper{}
}

func (query *WorldQueryWrapper) QueryCallback(proxyId int) bool {
	proxy := query.BroadPhase.GetUserData(proxyId).(*FixtureProxy)
	return query.Callback(proxy.Fixture)
}

func (world *World) QueryAABB(callback BroadPhaseQueryCallback, aabb AABB) {
	wrapper := MakeWorldQueryWrapper()
	wrapper.BroadPhase = &world.contactManager.broadPhase
	wrapper.Callback = callback
	world.contactManager.broadPhase.Query(wrapper.QueryCallback, aabb)
}

func (world *World) RayCast(callback RaycastCallback, point1 Vec2, point2 Vec2) {

	// TreeRayCastCallback
	wrapper := func(input RayCastInput, nodeId int) float64 {

		userData := world.contactManager.broadPhase.GetUserData(nodeId)
		proxy := userData.(*FixtureProxy)
		fixture := proxy.Fixture
		index := proxy.ChildIndex
		output := MakeRayCastOutput()
		hit := fixture.RayCast(&output, input, index)

		if hit {
			fraction := output.Fraction
			point := Vec2Add(Vec2MulScalar((1.0-fraction), input.P1), Vec2MulScalar(fraction, input.P2))
			return callback(fixture, point, output.Normal, fraction)
		}

		return input.MaxFraction
	}

	input := RayCastInput{}
	input.MaxFraction = 1.0
	input.P1 = point1
	input.P2 = point2
	world.contactManager.broadPhase.RayCast(wrapper, input)
}

func (world World) GetProxyCount() int {
	return world.contactManager.broadPhase.GetProxyCount()
}

func (world World) GetTreeHeight() int {
	return world.contactManager.broadPhase.GetTreeHeight()
}

func (world World) GetTreeBalance() int {
	return world.contactManager.broadPhase.GetTreeBalance()
}

func (world World) GetTreeQuality() float64 {
	return world.contactManager.broadPhase.GetTreeQuality()
}

func (world *World) ShiftOrigin(newOrigin Vec2) {

	assert(world.locked == false)
	if world.locked {
		return
	}

	for b := world.bodyList; b != nil; b = b.next {
		b.xf.P.OperatorMinusInplace(newOrigin)
		b.sweep.C0.OperatorMinusInplace(newOrigin)
		b.sweep.C.OperatorMinusInplace(newOrigin)
	}

	for j := world.jointList; j != nil; j = j.GetNext() {
		j.ShiftOrigin(newOrigin)
	}

	world.contactManager.broadPhase.ShiftOrigin(newOrigin)
}

func (world *World) Dump() {
	if world.locked {
		return
	}

	fmt.Print(fmt.Printf("b2Vec2 g(%.15f, %.15f);\n", world.gravity.X, world.gravity.Y))
	fmt.Print("m_world.SetGravity(g);\n")

	fmt.Print(fmt.Printf("b2Body** bodies = (b2Body**)b2Alloc(%d * sizeof(b2Body*));\n", world.bodyCount))
	//fmt.Print("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", m_jointCount)
	fmt.Print(fmt.Printf("b2Joint** joints = (b2Joint**)b2Alloc(%d * sizeof(b2Joint*));\n", world.jointCount))

	i := 0
	for b := world.bodyList; b != nil; b = b.next {
		b.islandIndex = i
		b.Dump()
		i++
	}

	i = 0
	for j := world.jointList; j != nil; j = j.GetNext() {
		j.SetIndex(i)
		i++
	}

	// First pass on joints, skip gear joints.
	for j := world.jointList; j != nil; j = j.GetNext() {
		if j.GetType() == GearJointType {
			continue
		}

		fmt.Print("{\n")
		j.Dump()
		fmt.Print("}\n")
	}

	// Second pass on joints, only gear joints.
	for j := world.jointList; j != nil; j = j.GetNext() {
		if j.GetType() != GearJointType {
			continue
		}

		fmt.Print("{\n")
		j.Dump()
		fmt.Print("}\n")
	}

	fmt.Print("b2Free(joints);\n")
	fmt.Print("b2Free(bodies);\n")
	fmt.Print("joints = nullptr;\n")
	fmt.Print("bodies = nullptr;\n")
}
