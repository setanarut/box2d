package b2

type EdgeAndCircleContact struct {
	Contact
}

func EdgeAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Edge)
	assert(fixtureB.Type() == Circle)
	res := &EdgeAndCircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func EdgeAndCircleContact_Destroy(contact IContact) { // should be a pointer
}

func (contact *EdgeAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	collideEdgeAndCircle(
		manifold,
		contact.GetFixtureA().Shape().(*EdgeShape), xfA,
		contact.GetFixtureB().Shape().(*CircleShape), xfB,
	)
}
