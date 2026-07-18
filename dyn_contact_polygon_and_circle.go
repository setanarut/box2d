package b2

type PolygonAndCircleContact struct {
	Contact
}

func PolygonAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Polygon)
	assert(fixtureB.Type() == Circle)
	res := &PolygonAndCircleContact{
		Contact: MakeContact(fixtureA, 0, fixtureB, 0),
	}

	return res
}

func PolygonAndCircleContact_Destroy(contact IContact) {}

func (contact *PolygonAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	collidePolygonAndCircle(
		manifold,
		contact.GetFixtureA().Shape().(*PolygonShape), xfA,
		contact.GetFixtureB().Shape().(*CircleShape), xfB,
	)
}
