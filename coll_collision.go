package b2

import (
	"math"
)

var ContactFeature_Type = struct {
	E_vertex uint8
	E_face   uint8
}{
	E_vertex: 0,
	E_face:   1,
}

// The features that intersect to form the contact point
// This must be 4 bytes or less.
type ContactFeature struct {
	IndexA uint8 ///< Feature index on shapeA
	IndexB uint8 ///< Feature index on shapeB
	TypeA  uint8 ///< The feature type on shapeA
	TypeB  uint8 ///< The feature type on shapeB
}

type ContactID ContactFeature

// Contact ids to facilitate warm starting.
// Used to quickly compare contact ids.
func (v ContactID) Key() uint32 {
	var key uint32 = 0
	key |= uint32(v.IndexA)
	key |= uint32(v.IndexB) << 8
	key |= uint32(v.TypeA) << 16
	key |= uint32(v.TypeB) << 24
	return key
}

func (v *ContactID) SetKey(key uint32) {
	(*v).IndexA = uint8(key & 0xFF)
	(*v).IndexB = byte(key >> 8 & 0xFF)
	(*v).TypeA = byte(key >> 16 & 0xFF)
	(*v).TypeB = byte(key >> 24 & 0xFF)
}

// A manifold point is a contact point belonging to a contact
// manifold. It holds details related to the geometry and dynamics
// of the contact points.
// The local point usage depends on the manifold type:
// -e_circles: the local center of circleB
// -e_faceA: the local center of cirlceB or the clip point of polygonB
// -e_faceB: the clip point of polygonA
// This structure is stored across time steps, so we keep it small.
// Note: the impulses are used for internal caching and may not
// provide reliable contact forces, especially for high speed collisions.
type ManifoldPoint struct {
	LocalPoint     Vec2      // usage depends on manifold type
	NormalImpulse  float64   // the non-penetration impulse
	TangentImpulse float64   // the friction impulse
	Id             ContactID // uniquely identifies a contact point between two shapes
}

// A manifold for two touching convex shapes.
// Box2D supports multiple types of contact:
// - clip point versus plane with radius
// - point versus point with radius (circles)
// The local point usage depends on the manifold type:
// -e_circles: the local center of circleA
// -e_faceA: the center of faceA
// -e_faceB: the center of faceB
// Similarly the local normal usage:
// -e_circles: not used
// -e_faceA: the normal on polygonA
// -e_faceB: the normal on polygonB
// We store contacts in this way so that position correction can
// account for movement, which is critical for continuous physics.
// All contact scenarios must be expressed in one of these types.
// This structure is stored across time steps, so we keep it small.

type ManifoldType uint8

const (
	Circles ManifoldType = 0
	FaceA   ManifoldType = 1
	FaceB   ManifoldType = 2
)

type Manifold struct {
	Type ManifoldType
	// the points of contact
	Points [maxManifoldPoints]ManifoldPoint
	// not use for Type::e_points
	LocalNormal Vec2
	// usage depends on manifold type
	LocalPoint Vec2
	// the number of manifold points
	PointCount int
}

func NewManifold() *Manifold {
	return &Manifold{}
}

// This is used to compute the current state of a contact manifold.
type WorldManifold struct {
	Normal      Vec2                       // world vector pointing from A to B
	Points      [maxManifoldPoints]Vec2    // world contact point (point of intersection)
	Separations [maxManifoldPoints]float64 // a negative value indicates overlap, in meters
}

var PointState = struct {
	NullState    uint8 // point does not exist
	AddState     uint8 // point was added in the update
	PersistState uint8 // point persisted across the update
	RemoveState  uint8 // point was removed in the update
}{
	NullState:    0,
	AddState:     1,
	PersistState: 2,
	RemoveState:  3,
}

// Used for computing contact manifolds.
type ClipVertex struct {
	V  Vec2
	Id ContactID
}

// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
type RayCastInput struct {
	P1, P2      Vec2
	MaxFraction float64
}

// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
// come from b2RayCastInput.
type RayCastOutput struct {
	Normal   Vec2
	Fraction float64
}

func MakeRayCastOutput() RayCastOutput {
	return RayCastOutput{}
}

// An axis aligned bounding box.
type AABB struct {
	LowerBound Vec2 // the lower vertex
	UpperBound Vec2 // the upper vertex
}

// Get the center of the AABB.
func (bb AABB) GetCenter() Vec2 {
	return Vec2MulScalar(
		0.5,
		Vec2Add(bb.LowerBound, bb.UpperBound),
	)
}

// Get the extents of the AABB (half-widths).
func (bb AABB) GetExtents() Vec2 {
	return Vec2MulScalar(
		0.5,
		Vec2Sub(bb.UpperBound, bb.LowerBound),
	)
}

// Get the perimeter length
func (bb AABB) GetPerimeter() float64 {
	wx := bb.UpperBound.X - bb.LowerBound.X
	wy := bb.UpperBound.Y - bb.LowerBound.Y
	return 2.0 * (wx + wy)
}

// Combine an AABB into this one.
func (bb *AABB) CombineInPlace(aabb AABB) {
	bb.LowerBound = Vec2Min(bb.LowerBound, aabb.LowerBound)
	bb.UpperBound = Vec2Max(bb.UpperBound, aabb.UpperBound)
}

// Combine two AABBs into this one.
func (bb *AABB) CombineTwoInPlace(aabb1, aabb2 AABB) {
	bb.LowerBound = Vec2Min(aabb1.LowerBound, aabb2.LowerBound)
	bb.UpperBound = Vec2Max(aabb1.UpperBound, aabb2.UpperBound)
}

// Does this aabb contain the provided AABB.
func (bb AABB) Contains(aabb AABB) bool {
	return (bb.LowerBound.X <= aabb.LowerBound.X &&
		bb.LowerBound.Y <= aabb.LowerBound.Y &&
		aabb.UpperBound.X <= bb.UpperBound.X &&
		aabb.UpperBound.Y <= bb.UpperBound.Y)
}

func (bb AABB) IsValid() bool {
	d := Vec2Sub(bb.UpperBound, bb.LowerBound)
	valid := d.X >= 0.0 && d.Y >= 0.0
	valid = valid && bb.LowerBound.IsValid() && bb.UpperBound.IsValid()
	return valid
}

func TestOverlapBoundingBoxes(a, b AABB) bool {

	d1 := Vec2Sub(b.LowerBound, a.UpperBound)
	d2 := Vec2Sub(a.LowerBound, b.UpperBound)

	if d1.X > 0.0 || d1.Y > 0.0 {
		return false
	}

	if d2.X > 0.0 || d2.Y > 0.0 {
		return false
	}

	return true
}

// Convex hull used for polygon collision
type Hull struct {
	Points [MaxPolygonVertices]Vec2
	Count  int
}

func (wm *WorldManifold) Initialize(mf *Manifold, xfA Transform, radiusA float64, xfB Transform, radiusB float64) {
	if mf.PointCount == 0 {
		return
	}

	switch mf.Type {
	case Circles:
		wm.Normal.Set(1.0, 0.0)
		pointA := TransformVec2Mul(xfA, mf.LocalPoint)
		pointB := TransformVec2Mul(xfB, mf.Points[0].LocalPoint)
		if Vec2DistanceSquared(pointA, pointB) > epsilon*epsilon {
			wm.Normal = Vec2Sub(pointB, pointA)
			wm.Normal.Normalize()
		}

		cA := Vec2Add(pointA, Vec2MulScalar(radiusA, wm.Normal))
		cB := Vec2Sub(pointB, Vec2MulScalar(radiusB, wm.Normal))

		wm.Points[0] = Vec2MulScalar(0.5, Vec2Add(cA, cB))
		wm.Separations[0] = Vec2Dot(Vec2Sub(cB, cA), wm.Normal)

	case FaceA:
		wm.Normal = RotVec2Mul(xfA.Q, mf.LocalNormal)
		planePoint := TransformVec2Mul(xfA, mf.LocalPoint)

		for i := 0; i < mf.PointCount; i++ {
			clipPoint := TransformVec2Mul(xfB, mf.Points[i].LocalPoint)
			cA := Vec2Add(
				clipPoint,
				Vec2MulScalar(
					radiusA-Vec2Dot(
						Vec2Sub(clipPoint, planePoint),
						wm.Normal,
					),
					wm.Normal,
				),
			)
			cB := Vec2Sub(clipPoint, Vec2MulScalar(radiusB, wm.Normal))
			wm.Points[i] = Vec2MulScalar(0.5, Vec2Add(cA, cB))
			wm.Separations[i] = Vec2Dot(
				Vec2Sub(cB, cA),
				wm.Normal,
			)
		}

	case FaceB:
		wm.Normal = RotVec2Mul(xfB.Q, mf.LocalNormal)
		planePoint := TransformVec2Mul(xfB, mf.LocalPoint)

		for i := 0; i < mf.PointCount; i++ {
			clipPoint := TransformVec2Mul(xfA, mf.Points[i].LocalPoint)
			cB := Vec2Add(clipPoint, Vec2MulScalar(
				radiusB-Vec2Dot(
					Vec2Sub(clipPoint, planePoint),
					wm.Normal,
				), wm.Normal,
			))
			cA := Vec2Sub(clipPoint, Vec2MulScalar(radiusA, wm.Normal))
			wm.Points[i] = Vec2MulScalar(0.5, Vec2Add(cA, cB))
			wm.Separations[i] = Vec2Dot(
				Vec2Sub(cA, cB),
				wm.Normal,
			)
		}

		// Ensure normal points from A to B.
		wm.Normal = wm.Normal.OperatorNegate()
	}
}

func GetPointStates(state1 *[maxManifoldPoints]uint8, state2 *[maxManifoldPoints]uint8, manifold1 Manifold, manifold2 Manifold) {

	for i := range maxManifoldPoints {
		state1[i] = PointState.NullState
		state2[i] = PointState.NullState
	}

	// Detect persists and removes.
	for i := 0; i < manifold1.PointCount; i++ {
		id := manifold1.Points[i].Id

		state1[i] = PointState.RemoveState

		for j := 0; j < manifold2.PointCount; j++ {
			if manifold2.Points[j].Id.Key() == id.Key() {
				state1[i] = PointState.PersistState
				break
			}
		}
	}

	// Detect persists and adds.
	for i := 0; i < manifold2.PointCount; i++ {
		id := manifold2.Points[i].Id

		state2[i] = PointState.AddState

		for j := 0; j < manifold1.PointCount; j++ {
			if manifold1.Points[j].Id.Key() == id.Key() {
				state2[i] = PointState.PersistState
				break
			}
		}
	}
}

// From Real-time Collision Detection, p179.
func (bb AABB) RayCast(output *RayCastOutput, input RayCastInput) bool {
	tmin := -maxFloat
	tmax := maxFloat

	p := input.P1
	d := Vec2Sub(input.P2, input.P1)
	absD := Vec2Abs(d)

	normal := Vec2{}

	for i := range 2 {
		if absD.OperatorIndexGet(i) < epsilon {
			// Parallel.
			if p.OperatorIndexGet(i) < bb.LowerBound.OperatorIndexGet(i) || bb.UpperBound.OperatorIndexGet(i) < p.OperatorIndexGet(i) {
				return false
			}
		} else {
			inv_d := 1.0 / d.OperatorIndexGet(i)
			t1 := (bb.LowerBound.OperatorIndexGet(i) - p.OperatorIndexGet(i)) * inv_d
			t2 := (bb.UpperBound.OperatorIndexGet(i) - p.OperatorIndexGet(i)) * inv_d

			// Sign of the normal vector.
			s := -1.0

			if t1 > t2 {
				t1, t2 = t2, t1
				s = 1.0
			}

			// Push the min up
			if t1 > tmin {
				normal.SetZero()
				normal.OperatorIndexSet(i, s)
				tmin = t1
			}

			// Pull the max down
			tmax = math.Min(tmax, t2)

			if tmin > tmax {
				return false
			}
		}
	}

	// Does the ray start inside the box?
	// Does the ray intersect beyond the max fraction?
	if tmin < 0.0 || input.MaxFraction < tmin {
		return false
	}

	// Intersection.
	output.Fraction = tmin
	output.Normal = normal
	return true
}

// Sutherland-Hodgman clipping.
func ClipSegmentToLine(vOut []ClipVertex, vIn []ClipVertex, normal Vec2, offset float64, vertexIndexA int) int {

	// Start with no output points
	count := 0

	// Calculate the distance of end points to the line
	distance0 := Vec2Dot(normal, vIn[0].V) - offset
	distance1 := Vec2Dot(normal, vIn[1].V) - offset

	// If the points are behind the plane
	if distance0 <= 0.0 {
		vOut[count] = vIn[0]
		count++
	}

	if distance1 <= 0.0 {
		vOut[count] = vIn[1]
		count++
	}

	// If the points are on different sides of the plane
	if distance0*distance1 < 0.0 {
		// Find intersection point of edge and plane
		interp := distance0 / (distance0 - distance1)
		vOut[count].V = Vec2Add(
			vIn[0].V,
			Vec2MulScalar(interp, Vec2Sub(vIn[1].V, vIn[0].V)),
		)

		// VertexA is hitting edgeB.
		vOut[count].Id.IndexA = uint8(vertexIndexA)
		vOut[count].Id.IndexB = vIn[0].Id.IndexB
		vOut[count].Id.TypeA = ContactFeature_Type.E_vertex
		vOut[count].Id.TypeB = ContactFeature_Type.E_face
		count++
	}

	return count
}

func TestOverlapShapes(shapeA IShape, indexA int, shapeB IShape, indexB int, xfA Transform, xfB Transform) bool {
	input := MakeDistanceInput()
	input.ProxyA.Set(shapeA, indexA)
	input.ProxyB.Set(shapeB, indexB)
	input.TransformA = xfA
	input.TransformB = xfB
	input.UseRadii = true

	cache := MakeSimplexCache()
	cache.Count = 0

	output := MakeDistanceOutput()

	Distance(&output, &cache, &input)

	return output.Distance < 10.0*epsilon
}

// quickhull recursion
func RecurseHull(p1 Vec2, p2 Vec2, ps []Vec2, count int) Hull {
	var hull Hull
	hull.Count = 0

	if count == 0 {
		return hull
	}

	// create an edge vector pointing from p1 to p2
	e := Vec2Sub(p2, p1)
	e.Normalize()

	// discard points left of e and find point furthest to the right of e
	rightPoints := make([]Vec2, MaxPolygonVertices)
	rightCount := 0

	bestIndex := 0
	bestDistance := Vec2Cross(Vec2Sub(ps[bestIndex], p1), e)
	if bestDistance > 0.0 {
		rightPoints[rightCount] = ps[bestIndex]
		rightCount++
	}

	for i := 1; i < count; i++ {
		distance := Vec2Cross(Vec2Sub(ps[i], p1), e)
		if distance > bestDistance {
			bestIndex = i
			bestDistance = distance
		}

		if distance > 0.0 {
			rightPoints[rightCount] = ps[i]
			rightCount++
		}
	}

	if bestDistance < 2.0*linearSlop {
		return hull
	}

	bestPoint := ps[bestIndex]

	// compute hull to the right of p1-bestPoint
	hull1 := RecurseHull(p1, bestPoint, rightPoints, rightCount)

	// compute hull to the right of bestPoint-p2
	hull2 := RecurseHull(bestPoint, p2, rightPoints, rightCount)

	// stich together hulls
	for i := 0; i < hull1.Count; i++ {
		hull.Points[hull.Count] = hull1.Points[i]
		hull.Count++
	}

	hull.Points[hull.Count] = bestPoint
	hull.Count++

	for i := 0; i < hull2.Count; i++ {
		hull.Points[hull.Count] = hull2.Points[i]
		hull.Count++
	}

	assert(hull.Count < MaxPolygonVertices)

	return hull
}

// Compute the convex hull of a set of points. Returns an empty hull if it fails.
// Some failure cases:
// - all points very close together
// - all points on a line
// - less than 3 points
// - more than b2_maxPolygonVertices points
// This welds close points and removes collinear points.
//
// quickhull algorithm
// - merges vertices based on b2_linearSlop
// - removes collinear points using b2_linearSlop
// - returns an empty hull if it fails
func ComputeHull(points []Vec2, count int) Hull {
	var hull Hull
	hull.Count = 0

	if count < 3 || count > MaxPolygonVertices {
		// check your data
		return hull
	}

	count = min(count, MaxPolygonVertices)

	aabb := AABB{
		LowerBound: Vec2{maxFloat, maxFloat},
		UpperBound: Vec2{-maxFloat, -maxFloat},
	}

	// Perform aggressive point welding. First point always remains.
	// Also compute the bounding box for later.
	var ps [MaxPolygonVertices]Vec2
	n := 0
	tolSqr := 16.0 * linearSlop * linearSlop
	for i := 0; i < count; i++ {
		aabb.LowerBound = Vec2Min(aabb.LowerBound, points[i])
		aabb.UpperBound = Vec2Max(aabb.UpperBound, points[i])

		vi := points[i]

		unique := true
		for j := 0; j < i; j++ {
			vj := points[j]

			distSqr := Vec2DistanceSquared(vi, vj)
			if distSqr < tolSqr {
				unique = false
				break
			}
		}

		if unique {
			ps[n] = vi
			n++
		}
	}

	if n < 3 {
		// all points very close together, check your data and check your scale
		return hull
	}

	// Find an extreme point as the first point on the hull
	c := aabb.GetCenter()
	i1 := 0
	dsq1 := Vec2DistanceSquared(c, ps[i1])
	for i := 1; i < n; i++ {
		dsq := Vec2DistanceSquared(c, ps[i])
		if dsq > dsq1 {
			i1 = i
			dsq1 = dsq
		}
	}

	// remove p1 from working set
	p1 := ps[i1]
	ps[i1] = ps[n-1]
	n = n - 1

	i2 := 0
	dsq2 := Vec2DistanceSquared(p1, ps[i2])
	for i := 1; i < n; i++ {
		dsq := Vec2DistanceSquared(p1, ps[i])
		if dsq > dsq2 {
			i2 = i
			dsq2 = dsq
		}
	}

	// remove p2 from working set
	p2 := ps[i2]
	ps[i2] = ps[n-1]
	n = n - 1

	// split the points into points that are left and right of the line p1-p2.
	rightPoints := make([]Vec2, MaxPolygonVertices-2)
	rightCount := 0

	leftPoints := make([]Vec2, MaxPolygonVertices-2)
	leftCount := 0

	e := Vec2Sub(p2, p1)
	e.Normalize()

	for i := 0; i < n; i++ {
		d := Vec2Cross(Vec2Sub(ps[i], p1), e)

		// slop used here to skip points that are very close to the line p1-p2
		if d >= 2.0*linearSlop {
			rightPoints[rightCount] = ps[i]
			rightCount++
		} else if d <= -2.0*linearSlop {
			leftPoints[leftCount] = ps[i]
			leftCount++
		}
	}

	// compute hulls on right and left
	hull1 := RecurseHull(p1, p2, rightPoints, rightCount)
	hull2 := RecurseHull(p2, p1, leftPoints, leftCount)

	if hull1.Count == 0 && hull2.Count == 0 {
		// all points collinear
		return hull
	}

	// stitch hulls together, preserving CCW winding order
	hull.Points[hull.Count] = p1
	hull.Count++

	for i := 0; i < hull1.Count; i++ {
		hull.Points[hull.Count] = hull1.Points[i]
		hull.Count++
	}

	hull.Points[hull.Count] = p2
	hull.Count++

	for i := 0; i < hull2.Count; i++ {
		hull.Points[hull.Count] = hull2.Points[i]
		hull.Count++
	}

	assert(hull.Count <= MaxPolygonVertices)

	// merge collinear
	searching := true
	for searching && hull.Count > 2 {
		searching = false

		for i := 0; i < hull.Count; i++ {
			i1 := i
			i2 := (i + 1) % hull.Count
			i3 := (i + 2) % hull.Count

			p1 := hull.Points[i1]
			p2 := hull.Points[i2]
			p3 := hull.Points[i3]

			e = Vec2Sub(p3, p1)
			e.Normalize()

			distance := Vec2Cross(Vec2Sub(p2, p1), e)
			if distance <= 2.0*linearSlop {
				// remove midpoint from hull
				for j := i2; j < hull.Count-1; j++ {
					hull.Points[j] = hull.Points[j+1]
				}
				hull.Count -= 1

				// continue searching for collinear points
				searching = true

				break
			}
		}
	}

	if hull.Count < 3 {
		// all points collinear, shouldn't be reached since this was validated above
		hull.Count = 0
	}

	return hull
}

// This determines if a hull is valid. Checks for:
// - convexity
// - collinear points
// This is expensive and should not be called at runtime.
func ValidateHull(hull *Hull) bool {
	if hull.Count < 3 || MaxPolygonVertices < hull.Count {
		return false
	}

	// test that every point is behind every edge
	for i := 0; i < hull.Count; i++ {
		// create an edge vector
		i1 := i
		var i2 int
		if i < hull.Count-1 {
			i2 = i1 + 1
		} else {
			i2 = 0
		}
		p := hull.Points[i1]
		e := Vec2Sub(hull.Points[i2], p)
		e.Normalize()

		for j := 0; j < hull.Count; j++ {
			// skip points that subtend the current edge
			if j == i1 || j == i2 {
				continue
			}

			distance := Vec2Cross(Vec2Sub(hull.Points[j], p), e)
			if distance >= 0.0 {
				return false
			}
		}
	}

	// test for collinear points
	for i := 0; i < hull.Count; i++ {
		i1 := i
		i2 := (i + 1) % hull.Count
		i3 := (i + 2) % hull.Count

		p1 := hull.Points[i1]
		p2 := hull.Points[i2]
		p3 := hull.Points[i3]

		e := Vec2Sub(p3, p1)
		e.Normalize()

		distance := Vec2Cross(Vec2Sub(p2, p1), e)
		if distance <= linearSlop {
			// p1-p2-p3 are collinear
			return false
		}
	}

	return true
}
