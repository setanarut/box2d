package b2

type ChainAndCircleContact struct {
	Contact
}

func ChainAndCircleContact_Create(fixtureA *Fixture, indexA int, fixtureB *Fixture, indexB int) IContact {
	assert(fixtureA.Type() == Chain)
	assert(fixtureB.Type() == Circle)
	res := &ChainAndCircleContact{
		Contact: MakeContact(fixtureA, indexA, fixtureB, indexB),
	}

	return res
}

func ChainAndCircleContact_Destroy(contact IContact) { // should be a pointer
}

func (contact *ChainAndCircleContact) Evaluate(manifold *Manifold, xfA Transform, xfB Transform) {

	chain := contact.GetFixtureA().Shape().(*ChainShape)
	edge := MakeEdgeShape()
	chain.GetChildEdge(&edge, contact.M_indexA)
	collideEdgeAndCircle(
		manifold,
		&edge, xfA,
		contact.GetFixtureB().Shape().(*CircleShape), xfB,
	)
}
