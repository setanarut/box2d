package b2

type ChainAndPolygonContact struct {
	Contact
}

func ChainAndPolygonContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Chain)
	assert(fixtureB.Type() == Polygon)
	res := &ChainAndPolygonContact{
		Contact: MakeContact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func ChainAndPolygonContact_Destroy(contact IContact) { // should be a pointer
}

func (contact *ChainAndPolygonContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {
	chain := contact.GetFixtureA().Shape().(*ChainShape)
	edge := MakeEdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	CollideEdgeAndPolygon(manifold, &edge, xfA, contact.GetFixtureB().Shape().(*PolygonShape), xfB)
}
