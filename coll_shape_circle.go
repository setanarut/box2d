package b2

import (
	"math"
)

// A solid circle shape
type CircleShape struct {
	Shape
	Pos Vec2
}

func MakeCircleShape() CircleShape {
	return CircleShape{
		Shape: Shape{
			ShapeType: Circle,
		},
	}
}

func NewCircleShape() *CircleShape {
	res := MakeCircleShape()
	return &res
}

///////////////////////////////////////////////////////////////////////////////

func (shape CircleShape) Clone() IShape {
	clone := NewCircleShape()
	clone.Radius = shape.Radius
	clone.Pos = shape.Pos
	return clone
}

func (shape CircleShape) GetChildCount() int {
	return 1
}

func (shape CircleShape) TestPoint(transform Transform, p Vec2) bool {
	center := Vec2Add(transform.P, RotVec2Mul(transform.Q, shape.Pos))
	d := Vec2Sub(p, center)
	return Vec2Dot(d, d) <= shape.Radius*shape.Radius
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
//
// @note because the circle is solid, rays that start inside do not hit because the normal is
// not defined.
func (shape CircleShape) RayCast(output *RayCastOutput, input RayCastInput, transform Transform, childIndex int) bool {

	position := Vec2Add(transform.P, RotVec2Mul(transform.Q, shape.Pos))
	s := Vec2Sub(input.P1, position)
	b := Vec2Dot(s, s) - shape.Radius*shape.Radius

	// Solve quadratic equation.
	r := Vec2Sub(input.P2, input.P1)
	c := Vec2Dot(s, r)
	rr := Vec2Dot(r, r)
	sigma := c*c - rr*b

	// Check for negative discriminant and short segment.
	if sigma < 0.0 || rr < epsilon {
		return false
	}

	// Find the point of intersection of the line with the circle.
	a := -(c + math.Sqrt(sigma))

	// Is the intersection point on the segment?
	if 0.0 <= a && a <= input.MaxFraction*rr {
		a /= rr
		output.Fraction = a
		output.Normal = Vec2Add(s, Vec2MulScalar(a, r))
		output.Normal.Normalize()
		return true
	}

	return false
}

func (shape CircleShape) ComputeAABB(aabb *AABB, transform Transform, childIndex int) {
	p := Vec2Add(transform.P, RotVec2Mul(transform.Q, shape.Pos))
	aabb.LowerBound.Set(p.X-shape.Radius, p.Y-shape.Radius)
	aabb.UpperBound.Set(p.X+shape.Radius, p.Y+shape.Radius)
}

func (shape CircleShape) ComputeMass(massData *MassData, density float64) {
	massData.Mass = density * pi * shape.Radius * shape.Radius
	massData.Center = shape.Pos

	// inertia about the local origin
	massData.I = massData.Mass * (0.5*shape.Radius*shape.Radius + Vec2Dot(shape.Pos, shape.Pos))
}

func (shape CircleShape) Destroy() {}
