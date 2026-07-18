package b2

// A line segment (edge) shape. These can be connected in chains or loops
// to other edge shapes. Edges created independently are two-sided and do
// no provide smooth movement across junctions.
type EdgeShape struct {
	Shape
	// These are the edge vertices
	M_vertex1, M_vertex2 Vec2

	// Optional adjacent vertices. These are used for smooth collision.
	M_vertex0, M_vertex3 Vec2

	// Uses m_vertex0 and m_vertex3 to create smooth collision.
	M_oneSided bool
}

func MakeEdgeShape() EdgeShape {
	return EdgeShape{
		Shape: Shape{
			ShapeType: Edge,
			Radius:    PolygonRadius,
		},
	}
}

func NewEdgeShape() *EdgeShape {
	res := MakeEdgeShape()
	return &res
}

// Set this as a part of a sequence. Vertex v0 precedes the edge and vertex v3
// follows. These extra vertices are used to provide smooth movement
// across junctions. This also makes the collision one-sided. The edge
// normal points to the right looking from v1 to v2.
func (edge *EdgeShape) SetOneSided(v0 Vec2, v1 Vec2, v2 Vec2, v3 Vec2) {
	edge.M_vertex0 = v0
	edge.M_vertex1 = v1
	edge.M_vertex2 = v2
	edge.M_vertex3 = v3
	edge.M_oneSided = true
}

// Set this as an isolated edge. Collision is two-sided.
func (edge *EdgeShape) SetTwoSided(v1 Vec2, v2 Vec2) {
	edge.M_vertex1 = v1
	edge.M_vertex2 = v2
	edge.M_oneSided = false
}

func (edge EdgeShape) Clone() IShape {
	clone := NewEdgeShape()
	clone.M_vertex0 = edge.M_vertex0
	clone.M_vertex1 = edge.M_vertex1
	clone.M_vertex2 = edge.M_vertex2
	clone.M_vertex3 = edge.M_vertex3
	clone.M_oneSided = edge.M_oneSided
	return clone
}

func (edge *EdgeShape) Destroy() {}

func (edge EdgeShape) GetChildCount() int {
	return 1
}

func (edge EdgeShape) TestPoint(xf Transform, p Vec2) bool {
	return false
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
func (edge EdgeShape) RayCast(output *RayCastOutput, input RayCastInput, xf Transform, childIndex int) bool {
	// Put the ray into the edge's frame of reference.
	p1 := RotVec2MulT(xf.Q, Vec2Sub(input.P1, xf.P))
	p2 := RotVec2MulT(xf.Q, Vec2Sub(input.P2, xf.P))
	d := Vec2Sub(p2, p1)

	v1 := edge.M_vertex1
	v2 := edge.M_vertex2
	e := Vec2Sub(v2, v1)

	// Normal points to the right, looking from v1 at v2
	normal := Vec2{e.Y, -e.X}
	normal.Normalize()

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	numerator := Vec2Dot(normal, Vec2Sub(v1, p1))
	if edge.M_oneSided && numerator > 0.0 {
		return false
	}

	denominator := Vec2Dot(normal, d)

	if denominator == 0.0 {
		return false
	}

	t := numerator / denominator
	if t < 0.0 || input.MaxFraction < t {
		return false
	}

	q := Vec2Add(p1, Vec2MulScalar(t, d))

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	r := Vec2Sub(v2, v1)
	rr := Vec2Dot(r, r)
	if rr == 0.0 {
		return false
	}

	s := Vec2Dot(Vec2Sub(q, v1), r) / rr
	if s < 0.0 || 1.0 < s {
		return false
	}

	output.Fraction = t
	if numerator > 0.0 {
		output.Normal = RotVec2Mul(xf.Q, normal).OperatorNegate()
	} else {
		output.Normal = RotVec2Mul(xf.Q, normal)
	}

	return true
}

func (edge EdgeShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int) {

	v1 := TransformVec2Mul(xf, edge.M_vertex1)
	v2 := TransformVec2Mul(xf, edge.M_vertex2)

	lower := Vec2Min(v1, v2)
	upper := Vec2Max(v1, v2)

	r := Vec2{edge.Radius, edge.Radius}
	aabb.LowerBound = Vec2Sub(lower, r)
	aabb.UpperBound = Vec2Sub(upper, r)
}

func (edge EdgeShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = 0.0
	massData.Center = Vec2MulScalar(0.5, Vec2Add(edge.M_vertex1, edge.M_vertex2))
	massData.I = 0.0
}
