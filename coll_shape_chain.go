package b2

// A chain shape is a free form sequence of line segments.
// The chain has one-sided collision, with the surface normal pointing to the right of the edge.
// This provides a counter-clockwise winding like the polygon shape.
// Connectivity information is used to create smooth collisions.
// @warning the chain will not collide properly if there are self-intersections.

// A circle shape.
type ChainShape struct {
	Shape

	// The vertices. Owned by this class.
	M_vertices []Vec2

	// The vertex count.
	M_count int

	M_prevVertex Vec2
	M_nextVertex Vec2
}

func MakeChainShape() ChainShape {
	return ChainShape{
		Shape: Shape{
			ShapeType: Chain,
			Radius:    PolygonRadius,
		},
		M_vertices: nil,
		M_count:    0,
	}
}

func (chain *ChainShape) Destroy() {
	chain.Clear()
}

func (chain *ChainShape) Clear() {
	chain.M_vertices = nil
	chain.M_count = 0
}

// Create a loop. This automatically adjusts connectivity.
// @param vertices an array of vertices, these are copied
// @param count the vertex count
func (chain *ChainShape) CreateLoop(vertices []Vec2, count int) {
	assert(chain.M_vertices == nil && chain.M_count == 0)
	assert(count >= 3)
	if count < 3 {
		return
	}

	for i := 1; i < count; i++ {
		v1 := vertices[i-1]
		v2 := vertices[i]
		// If the code crashes here, it means your vertices are too close together.
		assert(Vec2DistanceSquared(v1, v2) > linearSlop*linearSlop)
	}

	chain.M_count = count + 1
	chain.M_vertices = make([]Vec2, chain.M_count)
	copy(chain.M_vertices, vertices)

	chain.M_vertices[count] = chain.M_vertices[0]
	chain.M_prevVertex = chain.M_vertices[chain.M_count-2]
	chain.M_nextVertex = chain.M_vertices[1]
}

// Create a chain with ghost vertices to connect multiple chains together.
// @param vertices an array of vertices, these are copied
// @param count the vertex count
// @param prevVertex previous vertex from chain that connects to the start
// @param nextVertex next vertex from chain that connects to the end
func (chain *ChainShape) CreateChain(vertices []Vec2, count int, prevVertex Vec2, nextVertex Vec2) {
	assert(chain.M_vertices == nil && chain.M_count == 0)
	assert(count >= 2)
	for i := 1; i < count; i++ {
		// If the code crashes here, it means your vertices are too close together.
		assert(Vec2DistanceSquared(vertices[i-1], vertices[i]) > linearSlop*linearSlop)
	}

	chain.M_count = count
	chain.M_vertices = make([]Vec2, count)
	copy(chain.M_vertices, vertices)

	chain.M_prevVertex = prevVertex
	chain.M_nextVertex = nextVertex
}

func (chain ChainShape) Clone() IShape {
	clone := MakeChainShape()
	clone.CreateChain(chain.M_vertices, chain.M_count, chain.M_prevVertex, chain.M_nextVertex)
	return &clone
}

func (chain ChainShape) GetChildCount() int {
	// edge count = vertex count - 1
	return chain.M_count - 1
}

func (chain ChainShape) GetChildEdge(edge *EdgeShape, index int) {
	assert(0 <= index && index < chain.M_count-1)

	edge.ShapeType = Edge
	edge.Radius = chain.Radius

	edge.M_vertex1 = chain.M_vertices[index+0]
	edge.M_vertex2 = chain.M_vertices[index+1]
	edge.M_oneSided = true

	if index > 0 {
		edge.M_vertex0 = chain.M_vertices[index-1]
	} else {
		edge.M_vertex0 = chain.M_prevVertex
	}

	if index < chain.M_count-2 {
		edge.M_vertex3 = chain.M_vertices[index+2]
	} else {
		edge.M_vertex3 = chain.M_nextVertex
	}
}

func (chain ChainShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

func (chain ChainShape) RayCast(output *RayCastOutput, input RayCastInput, xf Transform, childIndex int) bool {
	assert(childIndex < chain.M_count)

	edgeShape := MakeEdgeShape()

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == chain.M_count {
		i2 = 0
	}

	edgeShape.M_vertex1 = chain.M_vertices[i1]
	edgeShape.M_vertex2 = chain.M_vertices[i2]

	return edgeShape.RayCast(output, input, xf, 0)
}

func (chain ChainShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int) {
	assert(childIndex < chain.M_count)

	i1 := childIndex
	i2 := childIndex + 1
	if i2 == chain.M_count {
		i2 = 0
	}

	v1 := TransformVec2Mul(xf, chain.M_vertices[i1])
	v2 := TransformVec2Mul(xf, chain.M_vertices[i2])

	lower := Vec2Min(v1, v2)
	upper := Vec2Max(v1, v2)

	r := Vec2{chain.Radius, chain.Radius}
	aabb.LowerBound = Vec2Sub(lower, r)
	aabb.UpperBound = Vec2Add(upper, r)
}

func (chain ChainShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = 0.0
	massData.Center.SetZero()
	massData.I = 0.0
}
