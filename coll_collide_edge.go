package b2

import (
	"math"
)

// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
func collideEdgeAndCircle(manifold *Manifold, edgeA *EdgeShape, xfA Transform, circleB *CircleShape, xfB Transform) {
	manifold.PointCount = 0

	// Compute circle in frame of edge
	Q := TransformVec2MulT(xfA, TransformVec2Mul(xfB, circleB.pos))

	A := edgeA.M_vertex1
	B := edgeA.M_vertex2
	e := Vec2Sub(B, A)

	// Normal points to the right for a CCW winding
	n := Vec2{e.Y, -e.X}
	offset := Vec2Dot(n, Vec2Sub(Q, A))

	oneSided := edgeA.M_oneSided
	if oneSided && offset < 0.0 {
		return
	}

	// Barycentric coordinates
	u := Vec2Dot(e, Vec2Sub(B, Q))
	v := Vec2Dot(e, Vec2Sub(Q, A))

	radius := edgeA.radius + circleB.radius

	cf := ContactFeature{}
	cf.IndexB = 0
	cf.TypeB = ContactFeature_Type.E_vertex

	// Region A
	if v <= 0.0 {
		P := A
		d := Vec2Sub(Q, P)
		dd := Vec2Dot(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to A?
		if edgeA.M_oneSided {
			A1 := edgeA.M_vertex0
			B1 := A
			e1 := Vec2Sub(B1, A1)
			u1 := Vec2Dot(e1, Vec2Sub(B1, Q))

			// Is the circle in Region AB of the previous edge?
			if u1 > 0.0 {
				return
			}
		}

		cf.IndexA = 0
		cf.TypeA = ContactFeature_Type.E_vertex
		manifold.PointCount = 1
		manifold.Type = Circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.SetKey(0)
		manifold.Points[0].Id.IndexA = cf.IndexA
		manifold.Points[0].Id.IndexB = cf.IndexB
		manifold.Points[0].Id.TypeA = cf.TypeA
		manifold.Points[0].Id.TypeB = cf.TypeB
		manifold.Points[0].LocalPoint = circleB.pos
		return
	}

	// Region B
	if u <= 0.0 {
		P := B
		d := Vec2Sub(Q, P)
		dd := Vec2Dot(d, d)
		if dd > radius*radius {
			return
		}

		// Is there an edge connected to B?
		if edgeA.M_oneSided {
			B2 := edgeA.M_vertex3
			A2 := B
			e2 := Vec2Sub(B2, A2)
			v2 := Vec2Dot(e2, Vec2Sub(Q, A2))

			// Is the circle in Region AB of the next edge?
			if v2 > 0.0 {
				return
			}
		}

		cf.IndexA = 1
		cf.TypeA = ContactFeature_Type.E_vertex
		manifold.PointCount = 1
		manifold.Type = Circles
		manifold.LocalNormal.SetZero()
		manifold.LocalPoint = P
		manifold.Points[0].Id.SetKey(0)
		manifold.Points[0].Id.IndexA = cf.IndexA
		manifold.Points[0].Id.IndexB = cf.IndexB
		manifold.Points[0].Id.TypeA = cf.TypeA
		manifold.Points[0].Id.TypeB = cf.TypeB
		manifold.Points[0].LocalPoint = circleB.pos
		return
	}

	// Region AB
	den := Vec2Dot(e, e)
	assert(den > 0.0)
	P := Vec2MulScalar(1.0/den, Vec2Add(Vec2MulScalar(u, A), Vec2MulScalar(v, B)))
	d := Vec2Sub(Q, P)
	dd := Vec2Dot(d, d)
	if dd > radius*radius {
		return
	}

	if offset < 0.0 {
		n.Set(-n.X, -n.Y)
	}
	n.Normalize()

	cf.IndexA = 0
	cf.TypeA = ContactFeature_Type.E_face
	manifold.PointCount = 1
	manifold.Type = FaceA
	manifold.LocalNormal = n
	manifold.LocalPoint = A
	manifold.Points[0].Id.SetKey(0)
	manifold.Points[0].Id.IndexA = cf.IndexA
	manifold.Points[0].Id.IndexB = cf.IndexB
	manifold.Points[0].Id.TypeA = cf.TypeA
	manifold.Points[0].Id.TypeB = cf.TypeB
	manifold.Points[0].LocalPoint = circleB.pos
}

// This structure is used to keep track of the best separating axis.
var EPAxis_Type = struct {
	E_unknown uint8
	E_edgeA   uint8
	E_edgeB   uint8
}{
	E_unknown: 0,
	E_edgeA:   1,
	E_edgeB:   2,
}

type ePAxis struct {
	Normal     Vec2
	Type       uint8
	Index      int
	Separation float64
}

func makeEPAxis() ePAxis {
	return ePAxis{}
}

// This holds polygon B expressed in frame A.
type tempPolygon struct {
	Vertices [MaxPolygonVertices]Vec2
	Normals  [MaxPolygonVertices]Vec2
	Count    int
}

// Reference face used for clipping
type referenceFace struct {
	I1, I2 int
	V1, V2 Vec2
	Normal Vec2

	SideNormal1 Vec2
	SideOffset1 float64

	SideNormal2 Vec2
	SideOffset2 float64
}

func ComputeEdgeSeparation(polygonB tempPolygon, v1 Vec2, normal1 Vec2) ePAxis {
	axis := makeEPAxis()
	axis.Type = EPAxis_Type.E_edgeA
	axis.Index = -1
	axis.Separation = -maxFloat
	axis.Normal.SetZero()

	var axes [2]Vec2 = [2]Vec2{normal1, normal1.OperatorNegate()}

	// Find axis with least overlap (min-max problem)
	for j := range 2 {
		sj := maxFloat

		// Find deepest polygon vertex along axis j
		for i := 0; i < polygonB.Count; i++ {
			si := Vec2Dot(axes[j], Vec2Sub(polygonB.Vertices[i], v1))
			if si < sj {
				sj = si
			}
		}

		if sj > axis.Separation {
			axis.Index = j
			axis.Separation = sj
			axis.Normal = axes[j]
		}
	}

	return axis
}

func ComputePolygonSeparation(polygonB tempPolygon, v1 Vec2, v2 Vec2) ePAxis {
	axis := makeEPAxis()
	axis.Type = EPAxis_Type.E_unknown
	axis.Index = -1
	axis.Separation = -maxFloat
	axis.Normal.SetZero()

	for i := 0; i < polygonB.Count; i++ {
		n := polygonB.Normals[i].OperatorNegate()

		s1 := Vec2Dot(n, Vec2Sub(polygonB.Vertices[i], v1))
		s2 := Vec2Dot(n, Vec2Sub(polygonB.Vertices[i], v2))
		s := math.Min(s1, s2)

		if s > axis.Separation {
			axis.Type = EPAxis_Type.E_edgeB
			axis.Index = i
			axis.Separation = s
			axis.Normal = n
		}
	}

	return axis
}

// Compute the collision manifold between an edge and a polygon.
func CollideEdgeAndPolygon(manifold *Manifold, edgeA *EdgeShape, xfA Transform, polygonB *PolygonShape, xfB Transform) {
	manifold.PointCount = 0

	xf := TransformMulT(xfA, xfB)

	centroidB := TransformVec2Mul(xf, polygonB.Centroid)

	v1 := edgeA.M_vertex1
	v2 := edgeA.M_vertex2

	edge1 := Vec2Sub(v2, v1)
	edge1.Normalize()

	// Normal points to the right for a CCW winding
	normal1 := Vec2{edge1.Y, -edge1.X}
	offset1 := Vec2Dot(normal1, Vec2Sub(centroidB, v1))

	oneSided := edgeA.M_oneSided
	if oneSided && offset1 < 0.0 {
		return
	}

	// Get polygonB in frameA
	var tempPolygonB tempPolygon
	tempPolygonB.Count = polygonB.Count
	for i := 0; i < polygonB.Count; i++ {
		tempPolygonB.Vertices[i] = TransformVec2Mul(xf, polygonB.Vertices[i])
		tempPolygonB.Normals[i] = RotVec2Mul(xf.Q, polygonB.Normals[i])
	}

	radius := polygonB.radius + edgeA.radius

	edgeAxis := ComputeEdgeSeparation(tempPolygonB, v1, normal1)
	if edgeAxis.Separation > radius {
		return
	}

	polygonAxis := ComputePolygonSeparation(tempPolygonB, v1, v2)
	if polygonAxis.Separation > radius {
		return
	}

	// Use hysteresis for jitter reduction.
	k_relativeTol := 0.98
	k_absoluteTol := 0.001

	primaryAxis := makeEPAxis()
	if polygonAxis.Separation-radius > k_relativeTol*(edgeAxis.Separation-radius)+k_absoluteTol {
		primaryAxis = polygonAxis
	} else {
		primaryAxis = edgeAxis
	}

	if oneSided {
		// Smooth collision
		// See https://box2d.org/posts/2020/06/ghost-collisions/

		edge0 := Vec2Sub(v1, edgeA.M_vertex0)
		edge0.Normalize()
		normal0 := Vec2{edge0.Y, -edge0.X}
		convex1 := Vec2Cross(edge0, edge1) >= 0.0

		edge2 := Vec2Sub(edgeA.M_vertex3, v2)
		edge2.Normalize()
		normal2 := Vec2{edge2.Y, -edge2.X}
		convex2 := Vec2Cross(edge1, edge2) >= 0.0

		sinTol := 0.1
		side1 := Vec2Dot(primaryAxis.Normal, edge1) <= 0.0

		// Check Gauss Map
		if side1 {
			if convex1 {
				if Vec2Cross(primaryAxis.Normal, normal0) > sinTol {
					// Skip region
					return
				}

				// Admit region
			} else {
				// Snap region
				primaryAxis = edgeAxis
			}
		} else {
			if convex2 {
				if Vec2Cross(normal2, primaryAxis.Normal) > sinTol {
					// Skip region
					return
				}

				// Admit region
			} else {
				// Snap region
				primaryAxis = edgeAxis
			}
		}
	}

	clipPoints := make([]ClipVertex, 2)
	ref := &referenceFace{}
	if primaryAxis.Type == EPAxis_Type.E_edgeA {
		manifold.Type = FaceA

		// Search for the polygon normal that is most anti-parallel to the edge normal.
		bestIndex := 0
		bestValue := Vec2Dot(primaryAxis.Normal, tempPolygonB.Normals[0])
		for i := 1; i < tempPolygonB.Count; i++ {
			value := Vec2Dot(primaryAxis.Normal, tempPolygonB.Normals[i])
			if value < bestValue {
				bestValue = value
				bestIndex = i
			}
		}

		i1 := bestIndex
		i2 := 0
		if i1+1 < tempPolygonB.Count {
			i2 = i1 + 1
		}

		clipPoints[0].V = tempPolygonB.Vertices[i1]
		clipPoints[0].Id.IndexA = 0
		clipPoints[0].Id.IndexB = uint8(i1)
		clipPoints[0].Id.TypeA = ContactFeature_Type.E_face
		clipPoints[0].Id.TypeB = ContactFeature_Type.E_vertex

		clipPoints[1].V = tempPolygonB.Vertices[i2]
		clipPoints[1].Id.IndexA = 0
		clipPoints[1].Id.IndexB = uint8(i2)
		clipPoints[1].Id.TypeA = ContactFeature_Type.E_face
		clipPoints[1].Id.TypeB = ContactFeature_Type.E_vertex

		ref.I1 = 0
		ref.I2 = 1
		ref.V1 = v1
		ref.V2 = v2
		ref.Normal = primaryAxis.Normal
		ref.SideNormal1 = edge1.OperatorNegate()
		ref.SideNormal2 = edge1
	} else {
		manifold.Type = FaceB

		clipPoints[0].V = v2
		clipPoints[0].Id.IndexA = 1
		clipPoints[0].Id.IndexB = uint8(primaryAxis.Index)
		clipPoints[0].Id.TypeA = ContactFeature_Type.E_vertex
		clipPoints[0].Id.TypeB = ContactFeature_Type.E_face

		clipPoints[1].V = v1
		clipPoints[1].Id.IndexA = 0
		clipPoints[1].Id.IndexB = uint8(primaryAxis.Index)
		clipPoints[1].Id.TypeA = ContactFeature_Type.E_vertex
		clipPoints[1].Id.TypeB = ContactFeature_Type.E_face

		ref.I1 = primaryAxis.Index
		ref.I2 = 0
		if ref.I1+1 < tempPolygonB.Count {
			ref.I2 = ref.I1 + 1
		}
		ref.V1 = tempPolygonB.Vertices[ref.I1]
		ref.V2 = tempPolygonB.Vertices[ref.I2]
		ref.Normal = tempPolygonB.Normals[ref.I1]

		// CCW winding
		ref.SideNormal1.Set(ref.Normal.Y, -ref.Normal.X)
		ref.SideNormal2 = ref.SideNormal1.OperatorNegate()
	}

	ref.SideOffset1 = Vec2Dot(ref.SideNormal1, ref.V1)
	ref.SideOffset2 = Vec2Dot(ref.SideNormal2, ref.V2)

	// Clip incident edge against reference face side planes
	clipPoints1 := make([]ClipVertex, 2)
	clipPoints2 := make([]ClipVertex, 2)
	np := 0

	// Clip to side 1
	np = ClipSegmentToLine(clipPoints1, clipPoints, ref.SideNormal1, ref.SideOffset1, ref.I1)

	if np < maxManifoldPoints {
		return
	}

	// Clip to side 2
	np = ClipSegmentToLine(clipPoints2, clipPoints1, ref.SideNormal2, ref.SideOffset2, ref.I2)

	if np < maxManifoldPoints {
		return
	}

	// Now clipPoints2 contains the clipped points.
	if primaryAxis.Type == EPAxis_Type.E_edgeA {
		manifold.LocalNormal = ref.Normal
		manifold.LocalPoint = ref.V1
	} else {
		manifold.LocalNormal = polygonB.Normals[ref.I1]
		manifold.LocalPoint = polygonB.Vertices[ref.I1]
	}

	pointCount := 0
	for i := range maxManifoldPoints {
		separation := 0.0

		separation = Vec2Dot(ref.Normal, Vec2Sub(clipPoints2[i].V, ref.V1))

		if separation <= radius {
			cp := &manifold.Points[pointCount]

			if primaryAxis.Type == EPAxis_Type.E_edgeA {
				cp.LocalPoint = TransformVec2MulT(xf, clipPoints2[i].V)
				cp.Id = clipPoints2[i].Id
			} else {
				cp.LocalPoint = clipPoints2[i].V
				cp.Id.TypeA = clipPoints2[i].Id.TypeB
				cp.Id.TypeB = clipPoints2[i].Id.TypeA
				cp.Id.IndexA = clipPoints2[i].Id.IndexB
				cp.Id.IndexB = clipPoints2[i].Id.IndexA
			}

			pointCount++
		}
	}

	manifold.PointCount = pointCount
}
