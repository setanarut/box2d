package b2

// A solid convex polygon. It is assumed that the interior of the polygon is to
// the left of each edge.
// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
// In most cases you should not need many vertices for a convex polygon.

type PolygonShape struct {
	Shape
	Centroid Vec2
	Vertices [MaxPolygonVertices]Vec2
	Normals  [MaxPolygonVertices]Vec2
	Count    int
}

func MakePolygonShape() PolygonShape {
	return PolygonShape{
		Shape: Shape{
			ShapeType: Polygon,
			Radius:    PolygonRadius,
		},
	}
}

func NewPolygonShape() *PolygonShape {
	res := MakePolygonShape()
	return &res
}

func (poly *PolygonShape) GetVertex(index int) *Vec2 {
	assert(0 <= index && index < poly.Count)
	return &poly.Vertices[index]
}

func (poly PolygonShape) Clone() IShape {

	clone := NewPolygonShape()
	clone.Centroid = poly.Centroid
	clone.Count = poly.Count

	for i, _ := range poly.Vertices {
		clone.Vertices[i] = poly.Vertices[i]
	}

	for i, _ := range poly.Normals {
		clone.Normals[i] = poly.Normals[i]
	}

	return clone
}

func (edge *PolygonShape) Destroy() {}

// SetAsBox builds vertices to represent an axis-aligned box centered on the local origin.
func (poly *PolygonShape) SetAsBox(halfWidth float64, halfHeight float64) {
	poly.Count = 4
	poly.Vertices[0].Set(-halfWidth, -halfHeight)
	poly.Vertices[1].Set(halfWidth, -halfHeight)
	poly.Vertices[2].Set(halfWidth, halfHeight)
	poly.Vertices[3].Set(-halfWidth, halfHeight)
	poly.Normals[0].Set(0.0, -1.0)
	poly.Normals[1].Set(1.0, 0.0)
	poly.Normals[2].Set(0.0, 1.0)
	poly.Normals[3].Set(-1.0, 0.0)
	poly.Centroid.SetZero()
}

func (poly *PolygonShape) SetAsBoxFromCenterAndAngle(hx float64, hy float64, center Vec2, angle float64) {
	poly.Count = 4
	poly.Vertices[0].Set(-hx, -hy)
	poly.Vertices[1].Set(hx, -hy)
	poly.Vertices[2].Set(hx, hy)
	poly.Vertices[3].Set(-hx, hy)
	poly.Normals[0].Set(0.0, -1.0)
	poly.Normals[1].Set(1.0, 0.0)
	poly.Normals[2].Set(0.0, 1.0)
	poly.Normals[3].Set(-1.0, 0.0)
	poly.Centroid = center

	xf := MakeTransform()
	xf.P = center
	xf.Q.Set(angle)

	// Transform vertices and normals.
	for i := 0; i < poly.Count; i++ {
		poly.Vertices[i] = TransformVec2Mul(xf, poly.Vertices[i])
		poly.Normals[i] = RotVec2Mul(xf.Q, poly.Normals[i])
	}
}

func (poly PolygonShape) GetChildCount() int {
	return 1
}

func ComputeCentroid(vs []Vec2, count int) Vec2 {

	assert(count >= 3)

	c := Vec2{}
	area := 0.0

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	s := vs[0]

	inv3 := 1.0 / 3.0

	for i := range count {
		// Triangle vertices.
		p1 := Vec2Sub(vs[0], s)
		p2 := Vec2Sub(vs[i], s)
		p3 := Vec2{}
		if i+1 < count {
			p3 = Vec2Sub(vs[i+1], s)
		} else {
			p3 = Vec2Sub(vs[0], s)
		}

		e1 := Vec2Sub(p2, p1)
		e2 := Vec2Sub(p3, p1)

		D := Vec2Cross(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		c.OperatorPlusInplace(Vec2MulScalar(triangleArea*inv3, Vec2Add(Vec2Add(p1, p2), p3)))
	}

	// Centroid
	assert(area > epsilon)
	c = Vec2Add(Vec2MulScalar(1.0/area, c), s)
	return c
}

// Create a convex hull from the given array of local points.
// The count must be in the range [3, b2_maxPolygonVertices].
// @warning the points may be re-ordered, even if they form a convex polygon
// @warning collinear points are handled but not removed. Collinear points
// may lead to poor stacking behavior.
func (poly *PolygonShape) Set(vertices []Vec2, count int) bool {
	hull := ComputeHull(vertices, count)

	if hull.Count < 3 {
		return false
	}

	poly.SetAsHull(hull)

	return true
}

// Create a polygon from a given convex hull (see b2ComputeHull).
// @warning the hull must be valid or this will crash or have unexpected behavior
func (poly *PolygonShape) SetAsHull(hull Hull) {
	assert(hull.Count >= 3)

	poly.Count = hull.Count

	// Copy vertices
	for i := 0; i < hull.Count; i++ {
		poly.Vertices[i] = hull.Points[i]
	}

	// Compute normals. Ensure the edges have non-zero length.
	for i := 0; i < poly.Count; i++ {
		i1 := i
		var i2 int
		if i+1 < poly.Count {
			i2 = i + 1
		} else {
			i2 = 0
		}
		edge := Vec2Sub(poly.Vertices[i2], poly.Vertices[i1])
		assert(edge.LengthSquared() > epsilon*epsilon)
		poly.Normals[i] = Vec2CrossVectorScalar(edge, 1.0)
		poly.Normals[i].Normalize()
	}

	// Compute the polygon centroid.
	poly.Centroid = ComputeCentroid(poly.Vertices[:], poly.Count)
}

func (poly PolygonShape) TestPoint(xf Transform, p Vec2) bool {
	pLocal := RotVec2MulT(xf.Q, Vec2Sub(p, xf.P))

	for i := 0; i < poly.Count; i++ {
		dot := Vec2Dot(poly.Normals[i], Vec2Sub(pLocal, poly.Vertices[i]))
		if dot > 0.0 {
			return false
		}
	}

	return true
}

// @note because the polygon is solid, rays that start inside do not hit because the normal is
// not defined.
func (poly PolygonShape) RayCast(output *RayCastOutput, input RayCastInput, xf Transform, childIndex int) bool {

	// Put the ray into the polygon's frame of reference.
	p1 := RotVec2MulT(xf.Q, Vec2Sub(input.P1, xf.P))
	p2 := RotVec2MulT(xf.Q, Vec2Sub(input.P2, xf.P))
	d := Vec2Sub(p2, p1)

	lower := 0.0
	upper := input.MaxFraction

	index := -1

	for i := 0; i < poly.Count; i++ {
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		numerator := Vec2Dot(poly.Normals[i], Vec2Sub(poly.Vertices[i], p1))
		denominator := Vec2Dot(poly.Normals[i], d)

		if denominator == 0.0 {
			if numerator < 0.0 {
				return false
			}
		} else {
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if denominator < 0.0 && numerator < lower*denominator {
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator
				index = i
			} else if denominator > 0.0 && numerator < upper*denominator {
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if upper < lower {
			return false
		}
	}

	assert(0.0 <= lower && lower <= input.MaxFraction)

	if index >= 0 {
		output.Fraction = lower
		output.Normal = RotVec2Mul(xf.Q, poly.Normals[index])
		return true
	}

	return false
}

func (poly PolygonShape) ComputeAABB(aabb *AABB, xf Transform, childIndex int) {

	lower := TransformVec2Mul(xf, poly.Vertices[0])
	upper := lower

	for i := 1; i < poly.Count; i++ {
		v := TransformVec2Mul(xf, poly.Vertices[i])
		lower = Vec2Min(lower, v)
		upper = Vec2Max(upper, v)
	}

	r := Vec2{poly.Radius, poly.Radius}
	aabb.LowerBound = Vec2Sub(lower, r)
	aabb.UpperBound = Vec2Sub(upper, r)
}

func (poly PolygonShape) ComputeMass(massData *MassData, density float64) {
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	assert(poly.Count >= 3)

	center := Vec2{}

	area := 0.0
	I := 0.0

	// Get a reference point for forming triangles.
	// Use the first vertex to reduce round-off errors.
	s := poly.Vertices[0]

	k_inv3 := 1.0 / 3.0

	for i := 0; i < poly.Count; i++ {
		// Triangle vertices.
		e1 := Vec2Sub(poly.Vertices[i], s)
		e2 := Vec2{}

		if i+1 < poly.Count {
			e2 = Vec2Sub(poly.Vertices[i+1], s)
		} else {
			e2 = Vec2Sub(poly.Vertices[0], s)
		}

		D := Vec2Cross(e1, e2)

		triangleArea := 0.5 * D
		area += triangleArea

		// Area weighted centroid
		center.OperatorPlusInplace(Vec2MulScalar(triangleArea*k_inv3, Vec2Add(e1, e2)))

		ex1 := e1.X
		ey1 := e1.Y
		ex2 := e2.X
		ey2 := e2.Y

		intx2 := ex1*ex1 + ex2*ex1 + ex2*ex2
		inty2 := ey1*ey1 + ey2*ey1 + ey2*ey2

		I += (0.25 * k_inv3 * D) * (intx2 + inty2)
	}

	// Total mass
	massData.Mass = density * area

	// Center of mass
	assert(area > epsilon)
	center.OperatorScalarMulInplace(1.0 / area)
	massData.Center = Vec2Add(center, s)

	// Inertia tensor relative to the local origin (point s).
	massData.I = density * I

	// Shift to center of mass then to original body origin.
	massData.I += massData.Mass * (Vec2Dot(massData.Center, massData.Center) - Vec2Dot(center, center))
}

func (poly PolygonShape) Validate() bool {
	if poly.Count < 3 || MaxPolygonVertices < poly.Count {
		return false
	}

	var hull Hull
	for i := 0; i < poly.Count; i++ {
		hull.Points[i] = poly.Vertices[i]
	}

	hull.Count = poly.Count

	return ValidateHull(&hull)
}
