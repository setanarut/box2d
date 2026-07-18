package b2

type EdgeAndPolygonContact struct {
	Contact
}

func EdgeAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Edge)
	assert(fixtureB.Type() == Polygon)
	res := &EdgeAndPolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func EdgeAndPolygonContact_Destroy(contact IContact) { // should be a pointer
}

func (contact *EdgeAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	CollideEdgeAndPolygon(
		manifold,
		contact.GetFixtureA().Shape().(*EdgeShape), xfA,
		contact.GetFixtureB().Shape().(*PolygonShape), xfB,
	)
}
