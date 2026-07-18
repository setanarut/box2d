package b2

// Find the max separation between poly1 and poly2 using edge normals from poly1.
func findMaxSeparation(edgeIndex *int, poly1 *PolygonShape, xf1 Transform, poly2 *PolygonShape, xf2 Transform) float64 {
	count1 := poly1.Count
	count2 := poly2.Count
	n1s := poly1.Normals
	v1s := poly1.Vertices
	v2s := poly2.Vertices

	xf := TransformMulT(xf2, xf1)

	bestIndex := 0
	maxSeparation := -maxFloat
	for i := range count1 {
		// Get poly1 normal in frame2.
		n := RotVec2Mul(xf.Q, n1s[i])
		v1 := TransformVec2Mul(xf, v1s[i])

		// Find deepest point for normal i.
		si := maxFloat
		for j := range count2 {
			sij := Vec2Dot(n, Vec2Sub(v2s[j], v1))
			if sij < si {
				si = sij
			}
		}

		if si > maxSeparation {
			maxSeparation = si
			bestIndex = i
		}
	}

	*edgeIndex = bestIndex
	return maxSeparation
}

func findIncidentEdge(c []ClipVertex, poly1 *PolygonShape, xf1 Transform, edge1 int, poly2 *PolygonShape, xf2 Transform) {

	normals1 := poly1.Normals

	count2 := poly2.Count
	vertices2 := poly2.Vertices
	normals2 := poly2.Normals

	assert(0 <= edge1 && edge1 < poly1.Count)

	// Get the normal of the reference edge in poly2's frame.
	normal1 := RotVec2MulT(xf2.Q, RotVec2Mul(xf1.Q, normals1[edge1]))

	// Find the incident edge on poly2.
	index := 0
	minDot := maxFloat
	for i := range count2 {
		dot := Vec2Dot(normal1, normals2[i])
		if dot < minDot {
			minDot = dot
			index = i
		}
	}

	// Build the clip vertices for the incident edge.
	i1 := index
	i2 := 0
	if i1+1 < count2 {
		i2 = i1 + 1
	}

	c[0].V = TransformVec2Mul(xf2, vertices2[i1])
	c[0].Id.IndexA = uint8(edge1)
	c[0].Id.IndexB = uint8(i1)
	c[0].Id.TypeA = ContactFeature_Type.E_face
	c[0].Id.TypeB = ContactFeature_Type.E_vertex

	c[1].V = TransformVec2Mul(xf2, vertices2[i2])
	c[1].Id.IndexA = uint8(edge1)
	c[1].Id.IndexB = uint8(i2)
	c[1].Id.TypeA = ContactFeature_Type.E_face
	c[1].Id.TypeB = ContactFeature_Type.E_vertex
}

// Find edge normal of max separation on A - return if separating axis is found
// Find edge normal of max separation on B - return if separation axis is found
// Choose reference edge as min(minA, minB)
// Find incident edge
// Clip

// The normal points from 1 to 2
func collidePolygons(manifold *Manifold, polyA *PolygonShape, xfA Transform, polyB *PolygonShape, xfB Transform) {

	manifold.PointCount = 0
	totalRadius := polyA.radius + polyB.radius

	edgeA := 0
	separationA := findMaxSeparation(&edgeA, polyA, xfA, polyB, xfB)
	if separationA > totalRadius {
		return
	}

	edgeB := 0
	separationB := findMaxSeparation(&edgeB, polyB, xfB, polyA, xfA)
	if separationB > totalRadius {
		return
	}

	var poly1 *PolygonShape // reference polygon
	var poly2 *PolygonShape // incident polygon

	xf1 := MakeTransform()
	xf2 := MakeTransform()

	edge1 := 0 // reference edge
	var flip uint8
	k_tol := 0.1 * linearSlop

	if separationB > separationA+k_tol {
		poly1 = polyB
		poly2 = polyA
		xf1 = xfB
		xf2 = xfA
		edge1 = edgeB
		manifold.Type = FaceB
		flip = 1
	} else {
		poly1 = polyA
		poly2 = polyB
		xf1 = xfA
		xf2 = xfB
		edge1 = edgeA
		manifold.Type = FaceA
		flip = 0
	}

	incidentEdge := make([]ClipVertex, 2)
	findIncidentEdge(incidentEdge, poly1, xf1, edge1, poly2, xf2)

	count1 := poly1.Count
	vertices1 := poly1.Vertices

	iv1 := edge1
	iv2 := 0
	if edge1+1 < count1 {
		iv2 = edge1 + 1
	}

	v11 := vertices1[iv1]
	v12 := vertices1[iv2]

	localTangent := Vec2Sub(v12, v11)
	localTangent.Normalize()

	localNormal := Vec2CrossVectorScalar(localTangent, 1.0)
	planePoint := Vec2MulScalar(0.5, Vec2Add(v11, v12))

	tangent := RotVec2Mul(xf1.Q, localTangent)
	normal := Vec2CrossVectorScalar(tangent, 1.0)

	v11 = TransformVec2Mul(xf1, v11)
	v12 = TransformVec2Mul(xf1, v12)

	// Face offset.
	frontOffset := Vec2Dot(normal, v11)

	// Side offsets, extended by polytope skin thickness.
	sideOffset1 := -Vec2Dot(tangent, v11) + totalRadius
	sideOffset2 := Vec2Dot(tangent, v12) + totalRadius

	// Clip incident edge against extruded edge1 side edges.
	clipPoints1 := make([]ClipVertex, 2)
	clipPoints2 := make([]ClipVertex, 2)
	np := 0

	// Clip to box side 1
	np = ClipSegmentToLine(clipPoints1, incidentEdge, tangent.OperatorNegate(), sideOffset1, iv1)

	if np < 2 {
		return
	}

	// Clip to negative box side 1
	np = ClipSegmentToLine(clipPoints2, clipPoints1, tangent, sideOffset2, iv2)

	if np < 2 {
		return
	}

	// Now clipPoints2 contains the clipped points.
	manifold.LocalNormal = localNormal
	manifold.LocalPoint = planePoint

	pointCount := 0
	for i := range maxManifoldPoints {
		separation := Vec2Dot(normal, clipPoints2[i].V) - frontOffset

		if separation <= totalRadius {
			cp := &manifold.Points[pointCount]
			cp.LocalPoint = TransformVec2MulT(xf2, clipPoints2[i].V)
			cp.Id = clipPoints2[i].Id
			if flip != 0 {
				// Swap features
				cf := cp.Id
				cp.Id.IndexA = cf.IndexB
				cp.Id.IndexB = cf.IndexA
				cp.Id.TypeA = cf.TypeB
				cp.Id.TypeB = cf.TypeA
			}
			pointCount++
		}
	}

	manifold.PointCount = pointCount
}
