package b2

import "math"

// @port(OK)
const Debug = false

// @port(OK)
func assert(a bool) {
	if !a {
		panic("Assert")
	}
}

const (
	maxFloat = math.MaxFloat64
	epsilon  = math.SmallestNonzeroFloat64
	pi       = math.Pi
)

// Global tuning constants based on meters-kilograms-seconds (MKS) units.
//
// Collision
const (
	// The maximum number of contact points between two convex shapes. Do
	// not change this value.
	maxManifoldPoints = 2

	// This is used to fatten AABBs in the dynamic tree. This allows proxies
	// to move by a small amount without triggering a tree adjustment.
	// This is in meters.
	aabbExtension = 0.1 * LengthUnitsPerMeter

	// This is used to fatten AABBs in the dynamic tree. This is used to predict
	// the future position based on the current displacement.
	// This is a dimensionless multiplier.
	aabbMultiplier = 4.0 * LengthUnitsPerMeter

	// A small length used as a collision and constraint tolerance. Usually it is
	// chosen to be numerically significant, but visually insignificant.
	linearSlop = 0.005 * LengthUnitsPerMeter

	// A small angle used as a collision and constraint tolerance. Usually it is
	// chosen to be numerically significant, but visually insignificant.
	angularSlop = (2.0 / 180.0 * pi)

	// The radius of the polygon/edge shape skin. This should not be modified. Making
	// this smaller means polygons will have an insufficient buffer for continuous collision.
	// Making it larger may create artifacts for vertex collision.
	PolygonRadius = (2.0 * linearSlop)

	// Maximum number of sub-steps per contact in continuous physics simulation.
	maxSubSteps = 8
)

// Dynamics
const (
	// Maximum number of contacts to be handled to solve a TOI impact.
	maxTOIContacts = 32

	// The maximum linear position correction used when solving constraints. This helps to
	// prevent overshoot.
	maxLinearCorrection = 0.2 * LengthUnitsPerMeter

	// The maximum angular position correction used when solving constraints. This helps to
	// prevent overshoot.
	maxAngularCorrection = (8.0 / 180.0 * pi)

	// The maximum linear translation of a body per step. This limit is very large and is used
	// to prevent numerical problems. You shouldn't need to adjust this. Meters.
	maxTranslation        = 2.0 * LengthUnitsPerMeter
	maxTranslationSquared = (maxTranslation * maxTranslation)

	// The maximum angular velocity of a body. This limit is very large and is used
	// to prevent numerical problems. You shouldn't need to adjust this.
	maxRotation        = (0.5 * pi)
	maxRotationSquared = (maxRotation * maxRotation)

	// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so
	// that overlap is removed in one time step. However using values close to 1 often lead
	// to overshoot.
	baumgarte    = 0.2
	toiBaumgarte = 0.75
)

// Sleep
const (
	// The time that a body must be still before it will go to sleep.
	timeToSleep = 0.5

	// A body cannot sleep if its linear velocity is above this tolerance.
	linearSleepTolerance = 0.01 * LengthUnitsPerMeter

	// A body cannot sleep if its angular velocity is above this tolerance.
	angularSleepTolerance = (2.0 / 180.0 * pi)
)
