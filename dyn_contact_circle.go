package b2

type CircleContact struct {
	Contact
}

func CircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Circle)
	assert(fixtureB.Type() == Circle)
	res := &CircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func CircleContact_Destroy(contact IContact) { // should be a pointer
}

func (contact *CircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	collideCircles(
		manifold,
		contact.GetFixtureA().Shape().(*CircleShape), xfA,
		contact.GetFixtureB().Shape().(*CircleShape), xfB,
	)
}
