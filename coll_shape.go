package b2

// This holds the mass data computed for a shape.
type MassData struct {
	// The mass of the shape, usually in kilograms.
	Mass float64

	// The position of the shape's centroid relative to the shape's origin.
	Center Vec2

	// The rotational inertia of the shape about the local origin.
	I float64
}

func MakeMassData() MassData {
	return MassData{}
}

func NewMassData() *MassData {
	res := MakeMassData()
	return &res
}

type ShapeType uint8

const (
	Circle ShapeType = iota
	Edge
	Polygon
	Chain
	typeCount
)

type IShape interface {
	Destroy()

	// Clone the concrete shape using the provided allocator.
	Clone() IShape

	// Get the type of this shape. You can use this to down cast to the concrete shape.
	// @return the shape type.
	GetType() ShapeType

	// Get the type of this shape. You can use this to down cast to the concrete shape.
	// @return the shape type.
	GetRadius() float64

	// Get the number of child primitives.
	GetChildCount() int

	// Test a point for containment in this shape. This only works for convex shapes.
	// @param xf the shape world transform.
	// @param p a point in world coordinates.
	TestPoint(xf Transform, p Vec2) bool

	// Cast a ray against a child shape.
	// @param output the ray-cast results.
	// @param input the ray-cast input parameters.
	// @param transform the transform to be applied to the shape.
	// @param childIndex the child shape index
	RayCast(output *RayCastOutput, input RayCastInput, transform Transform, childIndex int) bool

	// Given a transform, compute the associated axis aligned bounding box for a child shape.
	// @param aabb returns the axis aligned box.
	// @param xf the world transform of the shape.
	// @param childIndex the child shape
	ComputeAABB(aabb *AABB, xf Transform, childIndex int)

	// Compute the mass properties of this shape using its dimensions and density.
	// The inertia tensor is computed about the local origin.
	// @param massData returns the mass data for this shape.
	// @param density the density in kilograms per meter squared.
	ComputeMass(massData *MassData, density float64)
}

type Shape struct {
	ShapeType ShapeType
	// Radius of a shape. For polygonal shapes this must be b2_polygonRadius. There is no support for
	// making rounded polygons.
	Radius float64
}

func (shape Shape) GetType() ShapeType {
	return shape.ShapeType
}

func (shape Shape) GetRadius() float64 {
	return shape.Radius
}

// @addedgo
func (shape *Shape) SetRadius(r float64) {
	shape.Radius = r
}
