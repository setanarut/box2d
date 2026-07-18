package b2

// Settings that can be overriden for your application
//
// # Tunable Constants

// You can use this to change the length scale used by your game.
// For example for inches you could use 39.4
//
//	Default value 1.0
const LengthUnitsPerMeter float64 = 1.0

// The maximum number of vertices on a convex polygon. You cannot increase
// this too much because b2BlockAllocator has a maximum object size.
//
// Default value is 8
const MaxPolygonVertices int = 8
