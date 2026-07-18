package b2

import "math"

// A distance proxy is used by the GJK algorithm.
// It encapsulates any shape.
type DistanceProxy struct {
	M_buffer   [2]Vec2
	M_vertices []Vec2 // is a memory blob using pointer arithmetic in original implementation
	M_count    int
	M_radius   float64
}

func MakeDistanceProxy() DistanceProxy {
	return DistanceProxy{
		M_vertices: make([]Vec2, 0),
	}
}

// Used to warm start Distance.
// Set count to zero on first call.
type simplexCache struct {
	Metric float64 // length or area
	Count  int
	IndexA [3]int // vertices on shape A
	IndexB [3]int // vertices on shape B
}

func MakeSimplexCache() simplexCache {
	return simplexCache{
		Metric: 0,
		Count:  0,
		IndexA: [3]int{}, // vertices on shape A
		IndexB: [3]int{}, // vertices on shape B
	}
}

// Input for b2Distance.
// You have to option to use the shape radii
// in the computation. Even
type DistanceInput struct {
	ProxyA     DistanceProxy
	ProxyB     DistanceProxy
	TransformA Transform
	TransformB Transform
	UseRadii   bool
}

func MakeDistanceInput() DistanceInput {
	return DistanceInput{
		ProxyA:     MakeDistanceProxy(),
		ProxyB:     MakeDistanceProxy(),
		TransformA: MakeTransform(),
		TransformB: MakeTransform(),
		UseRadii:   false,
	}
}

// Output for b2Distance.
type DistanceOutput struct {
	PointA     Vec2 // closest point on shapeA
	PointB     Vec2 // closest point on shapeB
	Distance   float64
	Iterations int // number of GJK iterations used
}

func MakeDistanceOutput() DistanceOutput {
	return DistanceOutput{}
}

// Input parameters for b2ShapeCast
type ShapeCastInput struct {
	ProxyA       DistanceProxy
	ProxyB       DistanceProxy
	TransformA   Transform
	TransformB   Transform
	TranslationB Vec2
}

func MakeShapeCastInput() ShapeCastInput {
	return ShapeCastInput{
		ProxyA:     MakeDistanceProxy(),
		ProxyB:     MakeDistanceProxy(),
		TransformA: MakeTransform(),
		TransformB: MakeTransform(),
	}
}

// Output results for b2ShapeCast
type ShapeCastOutput struct {
	Point      Vec2
	Normal     Vec2
	Lambda     float64
	Iterations int
}

func (p DistanceProxy) GetVertexCount() int {
	return p.M_count
}

func (p DistanceProxy) GetVertex(index int) Vec2 {
	assert(0 <= index && index < p.M_count)
	return p.M_vertices[index]
}

func (p DistanceProxy) GetSupport(d Vec2) int {
	bestIndex := 0
	bestValue := Vec2Dot(p.M_vertices[0], d)
	for i := 1; i < p.M_count; i++ {
		value := Vec2Dot(p.M_vertices[i], d)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}

	return bestIndex
}

func (p DistanceProxy) GetSupportVertex(d Vec2) Vec2 {
	bestIndex := 0
	bestValue := Vec2Dot(p.M_vertices[0], d)

	for i := 1; i < p.M_count; i++ {
		value := Vec2Dot(p.M_vertices[i], d)
		if value > bestValue {
			bestIndex = i
			bestValue = value
		}
	}

	return p.M_vertices[bestIndex]
}

// GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
var b2_gjkCalls, b2_gjkIters, b2_gjkMaxIters int

// Initialize the proxy using the given shape. The shape
// must remain in scope while the proxy is in use.
func (p *DistanceProxy) Set(shape IShape, index int) {
	switch shape.GetType() {
	case Circle:
		circle := (shape).(*CircleShape)
		p.M_vertices = []Vec2{circle.Pos}
		p.M_count = 1
		p.M_radius = circle.Radius
	case Polygon:
		polygon := shape.(*PolygonShape)
		p.M_vertices = polygon.Vertices[:]
		p.M_count = polygon.Count
		p.M_radius = polygon.Radius
	case Chain:
		chain := shape.(*ChainShape)
		assert(0 <= index && index < chain.M_count)
		p.M_buffer[0] = chain.M_vertices[index]
		if index+1 < chain.M_count {
			p.M_buffer[1] = chain.M_vertices[index+1]
		} else {
			p.M_buffer[1] = chain.M_vertices[0]
		}
		p.M_vertices = p.M_buffer[:]
		p.M_count = 2
		p.M_radius = chain.Radius
	case Edge:
		edge := shape.(*EdgeShape)
		p.M_vertices = []Vec2{edge.M_vertex1, edge.M_vertex2}
		p.M_count = 2
		p.M_radius = edge.Radius
	default:
		assert(false)
	}
}

type SimplexVertex struct {
	WA     Vec2    // support point in proxyA
	WB     Vec2    // support point in proxyB
	W      Vec2    // wB - wA
	A      float64 // barycentric coordinate for closest point
	IndexA int     // wA index
	IndexB int     // wB index
}

func MakeSimplexVertex() SimplexVertex {
	return SimplexVertex{}
}

type Simplex struct {
	M_vs    [3]SimplexVertex
	M_count int
}

func MakeSimplex() Simplex {
	return Simplex{
		M_vs: [3]SimplexVertex{
			MakeSimplexVertex(),
			MakeSimplexVertex(),
			MakeSimplexVertex(),
		},
	}
}

func (simplex *Simplex) ReadCache(cache *simplexCache, proxyA *DistanceProxy, transformA Transform, proxyB *DistanceProxy, transformB Transform) {
	assert(cache.Count <= 3)

	// Copy data from cache.
	simplex.M_count = cache.Count
	vertices := &simplex.M_vs
	for i := 0; i < simplex.M_count; i++ {
		v := &vertices[i]
		v.IndexA = cache.IndexA[i]
		v.IndexB = cache.IndexB[i]
		wALocal := proxyA.GetVertex(v.IndexA)
		wBLocal := proxyB.GetVertex(v.IndexB)
		v.WA = TransformVec2Mul(transformA, wALocal)
		v.WB = TransformVec2Mul(transformB, wBLocal)
		v.W = Vec2Sub(v.WB, v.WA)
		v.A = 0.0
	}

	// Compute the new simplex metric, if it is substantially different than
	// old metric then flush the simplex.
	if simplex.M_count > 1 {
		metric1 := cache.Metric
		metric2 := simplex.GetMetric()
		if metric2 < 0.5*metric1 || 2.0*metric1 < metric2 || metric2 < epsilon {
			// Reset the simplex.
			simplex.M_count = 0
		}
	}

	// If the cache is empty or invalid ...
	if simplex.M_count == 0 {
		v := &vertices[0]
		v.IndexA = 0
		v.IndexB = 0
		wALocal := proxyA.GetVertex(0)
		wBLocal := proxyB.GetVertex(0)
		v.WA = TransformVec2Mul(transformA, wALocal)
		v.WB = TransformVec2Mul(transformB, wBLocal)
		v.W = Vec2Sub(v.WB, v.WA)
		v.A = 1.0
		simplex.M_count = 1
	}
}

func (simplex Simplex) WriteCache(cache *simplexCache) {
	cache.Metric = simplex.GetMetric()
	cache.Count = simplex.M_count
	vertices := &simplex.M_vs
	for i := 0; i < simplex.M_count; i++ {
		cache.IndexA[i] = vertices[i].IndexA
		cache.IndexB[i] = vertices[i].IndexB
	}
}

func (simplex Simplex) GetSearchDirection() Vec2 {
	switch simplex.M_count {
	case 1:
		return simplex.M_vs[0].W.OperatorNegate()

	case 2:
		{
			e12 := Vec2Sub(simplex.M_vs[1].W, simplex.M_vs[0].W)
			sgn := Vec2Cross(e12, simplex.M_vs[0].W.OperatorNegate())
			if sgn > 0.0 {
				// Origin is left of e12.
				return Vec2CrossScalarVector(1.0, e12)
			} else {
				// Origin is right of e12.
				return Vec2CrossVectorScalar(e12, 1.0)
			}
		}

	default:
		assert(false)
		return Vec2{}
	}
}

func (simplex Simplex) GetClosestPoint() Vec2 {
	switch simplex.M_count {
	case 0:
		assert(false)
		return Vec2{}

	case 1:
		return simplex.M_vs[0].W

	case 2:
		return Vec2Add(
			Vec2MulScalar(
				simplex.M_vs[0].A,
				simplex.M_vs[0].W,
			),
			Vec2MulScalar(
				simplex.M_vs[1].A,
				simplex.M_vs[1].W,
			),
		)

	case 3:
		return Vec2{}

	default:
		assert(false)
		return Vec2{}
	}
}

func (simplex Simplex) GetWitnessPoints(pA *Vec2, pB *Vec2) {
	switch simplex.M_count {
	case 0:
		assert(false)

	case 1:
		*pA = simplex.M_vs[0].WA
		*pB = simplex.M_vs[0].WB

	case 2:
		*pA = Vec2Add(
			Vec2MulScalar(simplex.M_vs[0].A, simplex.M_vs[0].WA),
			Vec2MulScalar(simplex.M_vs[1].A, simplex.M_vs[1].WA),
		)
		*pB = Vec2Add(
			Vec2MulScalar(simplex.M_vs[0].A, simplex.M_vs[0].WB),
			Vec2MulScalar(simplex.M_vs[1].A, simplex.M_vs[1].WB),
		)

	case 3:
		*pA = Vec2Add(
			Vec2Add(
				Vec2MulScalar(simplex.M_vs[0].A, simplex.M_vs[0].WA),
				Vec2MulScalar(simplex.M_vs[1].A, simplex.M_vs[1].WA),
			),
			Vec2MulScalar(simplex.M_vs[2].A, simplex.M_vs[2].WA),
		)
		*pB = *pA

	default:
		assert(false)
	}
}

func (simplex Simplex) GetMetric() float64 {
	switch simplex.M_count {
	case 0:
		assert(false)
		return 0.0

	case 1:
		return 0.0

	case 2:
		return Vec2Distance(simplex.M_vs[0].W, simplex.M_vs[1].W)

	case 3:
		return Vec2Cross(
			Vec2Sub(simplex.M_vs[1].W, simplex.M_vs[0].W),
			Vec2Sub(simplex.M_vs[2].W, simplex.M_vs[0].W),
		)

	default:
		assert(false)
		return 0.0
	}
}

////////////////////////////////////////////////////

// Solve a line segment using barycentric coordinates.
func (simplex *Simplex) Solve2() {
	w1 := simplex.M_vs[0].W
	w2 := simplex.M_vs[1].W
	e12 := Vec2Sub(w2, w1)

	// w1 region
	d12_2 := -Vec2Dot(w1, e12)
	if d12_2 <= 0.0 {
		// a2 <= 0, so we clamp it to 0
		simplex.M_vs[0].A = 1.0
		simplex.M_count = 1
		return
	}

	// w2 region
	d12_1 := Vec2Dot(w2, e12)
	if d12_1 <= 0.0 {
		// a1 <= 0, so we clamp it to 0
		simplex.M_vs[1].A = 1.0
		simplex.M_count = 1
		simplex.M_vs[0] = simplex.M_vs[1]
		return
	}

	// Must be in e12 region.
	inv_d12 := 1.0 / (d12_1 + d12_2)
	simplex.M_vs[0].A = d12_1 * inv_d12
	simplex.M_vs[1].A = d12_2 * inv_d12
	simplex.M_count = 2
}

// Possible regions:
// - points[2]
// - edge points[0]-points[2]
// - edge points[1]-points[2]
// - inside the triangle
func (simplex *Simplex) Solve3() {

	w1 := simplex.M_vs[0].W
	w2 := simplex.M_vs[1].W
	w3 := simplex.M_vs[2].W

	// Edge12
	// [1      1     ][a1] = [1]
	// [w1.e12 w2.e12][a2] = [0]
	// a3 = 0
	e12 := Vec2Sub(w2, w1)
	w1e12 := Vec2Dot(w1, e12)
	w2e12 := Vec2Dot(w2, e12)
	d12_1 := w2e12
	d12_2 := -w1e12

	// Edge13
	// [1      1     ][a1] = [1]
	// [w1.e13 w3.e13][a3] = [0]
	// a2 = 0
	e13 := Vec2Sub(w3, w1)
	w1e13 := Vec2Dot(w1, e13)
	w3e13 := Vec2Dot(w3, e13)
	d13_1 := w3e13
	d13_2 := -w1e13

	// Edge23
	// [1      1     ][a2] = [1]
	// [w2.e23 w3.e23][a3] = [0]
	// a1 = 0
	e23 := Vec2Sub(w3, w2)
	w2e23 := Vec2Dot(w2, e23)
	w3e23 := Vec2Dot(w3, e23)
	d23_1 := w3e23
	d23_2 := -w2e23

	// Triangle123
	n123 := Vec2Cross(e12, e13)

	d123_1 := n123 * Vec2Cross(w2, w3)
	d123_2 := n123 * Vec2Cross(w3, w1)
	d123_3 := n123 * Vec2Cross(w1, w2)

	// w1 region
	if d12_2 <= 0.0 && d13_2 <= 0.0 {
		simplex.M_vs[0].A = 1.0
		simplex.M_count = 1
		return
	}

	// e12
	if d12_1 > 0.0 && d12_2 > 0.0 && d123_3 <= 0.0 {
		inv_d12 := 1.0 / (d12_1 + d12_2)
		simplex.M_vs[0].A = d12_1 * inv_d12
		simplex.M_vs[1].A = d12_2 * inv_d12
		simplex.M_count = 2
		return
	}

	// e13
	if d13_1 > 0.0 && d13_2 > 0.0 && d123_2 <= 0.0 {
		inv_d13 := 1.0 / (d13_1 + d13_2)
		simplex.M_vs[0].A = d13_1 * inv_d13
		simplex.M_vs[2].A = d13_2 * inv_d13
		simplex.M_count = 2
		simplex.M_vs[1] = simplex.M_vs[2]
		return
	}

	// w2 region
	if d12_1 <= 0.0 && d23_2 <= 0.0 {
		simplex.M_vs[1].A = 1.0
		simplex.M_count = 1
		simplex.M_vs[0] = simplex.M_vs[1]
		return
	}

	// w3 region
	if d13_1 <= 0.0 && d23_1 <= 0.0 {
		simplex.M_vs[2].A = 1.0
		simplex.M_count = 1
		simplex.M_vs[0] = simplex.M_vs[2]
		return
	}

	// e23
	if d23_1 > 0.0 && d23_2 > 0.0 && d123_1 <= 0.0 {
		inv_d23 := 1.0 / (d23_1 + d23_2)
		simplex.M_vs[1].A = d23_1 * inv_d23
		simplex.M_vs[2].A = d23_2 * inv_d23
		simplex.M_count = 2
		simplex.M_vs[0] = simplex.M_vs[2]
		return
	}

	// Must be in triangle123
	inv_d123 := 1.0 / (d123_1 + d123_2 + d123_3)
	simplex.M_vs[0].A = d123_1 * inv_d123
	simplex.M_vs[1].A = d123_2 * inv_d123
	simplex.M_vs[2].A = d123_3 * inv_d123
	simplex.M_count = 3
}

func Distance(output *DistanceOutput, cache *simplexCache, input *DistanceInput) {
	b2_gjkCalls++

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	transformA := input.TransformA
	transformB := input.TransformB

	// Initialize the simplex.
	simplex := MakeSimplex()
	simplex.ReadCache(cache, proxyA, transformA, proxyB, transformB)

	// Get simplex vertices as an array.
	vertices := &simplex.M_vs
	k_maxIters := 20

	// These store the vertices of the last simplex so that we
	// can check for duplicates and prevent cycling.
	saveA := make([]int, 3)
	saveB := make([]int, 3)
	saveCount := 0

	// Main iteration loop.
	iter := 0
	for iter < k_maxIters {
		// Copy simplex so we can identify duplicates.
		saveCount = simplex.M_count
		for i := 0; i < saveCount; i++ {
			saveA[i] = vertices[i].IndexA
			saveB[i] = vertices[i].IndexB
		}

		switch simplex.M_count {
		case 1:

		case 2:
			simplex.Solve2()

		case 3:
			simplex.Solve3()

		default:
			assert(false)
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.M_count == 3 {
			break
		}

		// Get search direction.
		d := simplex.GetSearchDirection()

		// Ensure the search direction is numerically fit.
		if d.LengthSquared() < epsilon*epsilon {
			// The origin is probably contained by a line segment
			// or triangle. Thus the shapes are overlapped.

			// We can't return zero here even though there may be overlap.
			// In case the simplex is a point, segment, or triangle it is difficult
			// to determine if the origin is contained in the CSO or very close to it.
			break
		}

		// Compute a tentative new simplex vertex using support points.
		vertex := &vertices[simplex.M_count]
		vertex.IndexA = proxyA.GetSupport(
			RotVec2MulT(transformA.Q, d.OperatorNegate()),
		)
		vertex.WA = TransformVec2Mul(transformA, proxyA.GetVertex(vertex.IndexA))
		vertex.IndexB = proxyB.GetSupport(RotVec2MulT(transformB.Q, d))
		vertex.WB = TransformVec2Mul(transformB, proxyB.GetVertex(vertex.IndexB))
		vertex.W = Vec2Sub(vertex.WB, vertex.WA)

		// Iteration count is equated to the number of support point calls.
		iter++
		b2_gjkIters++

		// Check for duplicate support points. This is the main termination criteria.
		duplicate := false
		for i := 0; i < saveCount; i++ {
			if vertex.IndexA == saveA[i] && vertex.IndexB == saveB[i] {
				duplicate = true
				break
			}
		}

		// If we found a duplicate support point we must exit to avoid cycling.
		if duplicate {
			break
		}

		// New vertex is ok and needed.
		simplex.M_count++
	}

	if iter > b2_gjkMaxIters {
		b2_gjkMaxIters = iter
	}

	// Prepare output.
	simplex.GetWitnessPoints(&output.PointA, &output.PointB)
	output.Distance = Vec2Distance(output.PointA, output.PointB)
	output.Iterations = iter

	// Cache the simplex.
	simplex.WriteCache(cache)

	// Apply radii if requested.
	if input.UseRadii {
		if output.Distance < epsilon {
			// Shapes are too close to safely compute normal
			p := Vec2MulScalar(
				0.5,
				Vec2Add(output.PointA, output.PointB),
			)
			output.PointA = p
			output.PointB = p
			output.Distance = 0.0
		} else {
			// Keep closest points on perimeter even if overlapped, this way
			// the points move smoothly.
			rA := proxyA.M_radius
			rB := proxyB.M_radius
			normal := Vec2Sub(output.PointB, output.PointA)
			normal.Normalize()
			output.Distance = math.Max(0.0, output.Distance-rA-rB)
			output.PointA.OperatorPlusInplace(
				Vec2MulScalar(rA, normal),
			)
			output.PointB.OperatorMinusInplace(
				Vec2MulScalar(rB, normal),
			)
		}
	}
}

// GJK-raycast
// Algorithm by Gino van den Bergen.
// "Smooth Mesh Contacts with GJK" in Game Physics Pearls. 2010
//
// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
// @returns true if hit, false if there is no hit or an initial overlap
func ShapeCast(output *ShapeCastOutput, input *ShapeCastInput) bool {
	output.Iterations = 0
	output.Lambda = 1.0
	output.Normal.SetZero()
	output.Point.SetZero()

	proxyA := &input.ProxyA
	proxyB := &input.ProxyB

	radiusA := math.Max(proxyA.M_radius, PolygonRadius)
	radiusB := math.Max(proxyB.M_radius, PolygonRadius)
	radius := radiusA + radiusB

	xfA := input.TransformA
	xfB := input.TransformB

	r := input.TranslationB
	n := Vec2{}
	var lambda float64 = 0.0

	// Initial simplex
	simplex := MakeSimplex()
	simplex.M_count = 0

	// Get simplex vertices as an array.
	vertices := &simplex.M_vs
	//b2SimplexVertex* vertices = &simplex.m_v1;

	// Get support point in -r direction
	indexA := proxyA.GetSupport(RotVec2MulT(xfA.Q, r.OperatorNegate()))
	wA := TransformVec2Mul(xfA, proxyA.GetVertex(indexA))
	indexB := proxyB.GetSupport(RotVec2MulT(xfB.Q, r))
	wB := TransformVec2Mul(xfB, proxyB.GetVertex(indexB))
	v := Vec2Sub(wA, wB)

	// Sigma is the target distance between polygons
	sigma := math.Max(PolygonRadius, radius-PolygonRadius)
	var tolerance float64 = 0.5 * linearSlop

	// Main iteration loop.
	k_maxIters := 20
	iter := 0
	for iter < k_maxIters && v.Length()-sigma > tolerance {
		assert(simplex.M_count < 3)

		output.Iterations += 1

		// Support in direction -v (A - B)
		indexA = proxyA.GetSupport(RotVec2MulT(xfA.Q, v.OperatorNegate()))
		wA = TransformVec2Mul(xfA, proxyA.GetVertex(indexA))
		indexB = proxyB.GetSupport(RotVec2MulT(xfB.Q, v))
		wB = TransformVec2Mul(xfB, proxyB.GetVertex(indexB))
		p := Vec2Sub(wA, wB)

		// -v is a normal at p
		v.Normalize()

		// Intersect ray with plane
		vp := Vec2Dot(v, p)
		vr := Vec2Dot(v, r)
		if vp-sigma > lambda*vr {
			if vr <= 0.0 {
				return false
			}

			lambda = (vp - sigma) / vr
			if lambda > 1.0 {
				return false
			}

			n = v.OperatorNegate()
			simplex.M_count = 0
		}

		// Reverse simplex since it works with B - A.
		// Shift by lambda * r because we want the closest point to the current clip point.
		// Note that the support point p is not shifted because we want the plane equation
		// to be formed in unshifted space.
		vertex := &vertices[simplex.M_count]
		vertex.IndexA = indexB
		vertex.WA = Vec2Add(wB, Vec2MulScalar(lambda, r))
		vertex.IndexB = indexA
		vertex.WB = wA
		vertex.W = Vec2Sub(vertex.WB, vertex.WA)
		vertex.A = 1.0
		simplex.M_count += 1

		switch simplex.M_count {
		case 1:

		case 2:
			simplex.Solve2()

		case 3:
			simplex.Solve3()

		default:
			assert(false)
		}

		// If we have 3 points, then the origin is in the corresponding triangle.
		if simplex.M_count == 3 {
			// Overlap
			return false
		}

		// Get search direction.
		v = simplex.GetClosestPoint()

		// Iteration count is equated to the number of support point calls.
		iter++
	}

	if iter == 0 {
		// Initial overlap
		return false
	}

	// Prepare output.
	pointA := Vec2{}
	pointB := Vec2{}
	simplex.GetWitnessPoints(&pointB, &pointA)

	if v.LengthSquared() > 0.0 {
		n = v.OperatorNegate()
		n.Normalize()
	}

	output.Point = Vec2Add(pointA, Vec2MulScalar(radiusA, n))
	output.Normal = n
	output.Lambda = lambda
	output.Iterations = iter
	return true
}
