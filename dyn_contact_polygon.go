package b2

type PolygonContact struct {
	Contact
}

func PolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Polygon)
	assert(fixtureB.Type() == Polygon)
	res := &PolygonContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func PolygonContact_Destroy(contact IContact) { // should be a pointer
}

func (contact *PolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	collidePolygons(
		manifold,
		contact.GetFixtureA().Shape().(*PolygonShape), xfA,
		contact.GetFixtureB().Shape().(*PolygonShape), xfB,
	)
}
