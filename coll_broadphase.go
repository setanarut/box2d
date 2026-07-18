package b2

type BroadPhaseAddPairCallback func(userDataA any, userDataB any)

type Pair struct {
	ProxyIdA int
	ProxyIdB int
}

const E_nullProxy = -1

type BroadPhase struct {
	M_tree DynamicTree

	M_proxyCount int

	M_moveBuffer   []int
	M_moveCapacity int
	M_moveCount    int

	M_pairBuffer   []Pair
	M_pairCapacity int
	M_pairCount    int

	M_queryProxyId int
}

// Was used for sorting the pair buffer to expose duplicates:
// sort.Sort(PairByLessThan(bp.M_pairBuffer[:bp.M_pairCount]))
type PairByLessThan []Pair

func (a PairByLessThan) Len() int      { return len(a) }
func (a PairByLessThan) Swap(i, j int) { a[i], a[j] = a[j], a[i] }
func (a PairByLessThan) Less(i, j int) bool {
	if a[i].ProxyIdA < a[j].ProxyIdA {
		return true
	}

	if a[i].ProxyIdA == a[j].ProxyIdA {
		return a[i].ProxyIdB < a[j].ProxyIdB
	}

	return false
}

func (bp BroadPhase) GetUserData(proxyId int) any {
	return bp.M_tree.GetUserData(proxyId)
}

func (bp BroadPhase) TestOverlap(proxyIdA int, proxyIdB int) bool {
	return TestOverlapBoundingBoxes(
		bp.M_tree.GetFatAABB(proxyIdA),
		bp.M_tree.GetFatAABB(proxyIdB),
	)
}

func (bp BroadPhase) GetFatAABB(proxyId int) AABB {
	return bp.M_tree.GetFatAABB(proxyId)
}

func (bp BroadPhase) GetProxyCount() int {
	return bp.M_proxyCount
}

func (bp BroadPhase) GetTreeHeight() int {
	return bp.M_tree.GetHeight()
}

func (bp BroadPhase) GetTreeBalance() int {
	return bp.M_tree.GetMaxBalance()
}

func (bp BroadPhase) GetTreeQuality() float64 {
	return bp.M_tree.GetAreaRatio()
}

func (bp *BroadPhase) UpdatePairs(addPairCallback BroadPhaseAddPairCallback) {
	// Reset pair buffer
	bp.M_pairCount = 0

	// Perform tree queries for all moving proxies.
	for i := 0; i < bp.M_moveCount; i++ {
		bp.M_queryProxyId = bp.M_moveBuffer[i]
		if bp.M_queryProxyId == E_nullProxy {
			continue
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		fatAABB := bp.M_tree.GetFatAABB(bp.M_queryProxyId)

		// Query tree, create pairs and add them pair buffer.
		bp.M_tree.Query(bp.QueryCallback, fatAABB)
	}

	// Send pairs to caller
	for i := 0; i < bp.M_pairCount; i++ {
		primaryPair := bp.M_pairBuffer[i]
		userDataA := bp.M_tree.GetUserData(primaryPair.ProxyIdA)
		userDataB := bp.M_tree.GetUserData(primaryPair.ProxyIdB)

		addPairCallback(userDataA, userDataB)
	}

	// Clear move flags
	for i := 0; i < bp.M_moveCount; i++ {
		proxyId := bp.M_moveBuffer[i]
		if proxyId == E_nullProxy {
			continue
		}
		bp.M_tree.ClearMoved(proxyId)
	}

	// Reset move buffer
	bp.M_moveCount = 0
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
// BroadPhase.cpp
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

func MakeBroadPhase() BroadPhase {

	pairCapacity := 16
	moveCapacity := 16

	tree := MakeDynamicTree()

	return BroadPhase{
		M_tree:       tree,
		M_proxyCount: 0,

		M_pairCapacity: pairCapacity,
		M_pairCount:    0,
		M_pairBuffer:   make([]Pair, pairCapacity),

		M_moveCapacity: moveCapacity,
		M_moveCount:    0,
		M_moveBuffer:   make([]int, moveCapacity),
	}
}

func (bp *BroadPhase) CreateProxy(aabb AABB, userData any) int {
	proxyId := bp.M_tree.CreateProxy(aabb, userData)
	bp.M_proxyCount++
	bp.BufferMove(proxyId)
	return proxyId
}

func (bp *BroadPhase) DestroyProxy(proxyId int) {
	bp.UnBufferMove(proxyId)
	bp.M_proxyCount--
	bp.M_tree.DestroyProxy(proxyId)
}

func (bp *BroadPhase) MoveProxy(proxyId int, aabb AABB, displacement Vec2) {
	buffer := bp.M_tree.MoveProxy(proxyId, aabb, displacement)
	if buffer {
		bp.BufferMove(proxyId)
	}
}

func (bp *BroadPhase) TouchProxy(proxyId int) {
	bp.BufferMove(proxyId)
}

func (bp *BroadPhase) BufferMove(proxyId int) {
	if bp.M_moveCount == bp.M_moveCapacity {
		bp.M_moveBuffer = append(bp.M_moveBuffer, make([]int, bp.M_moveCapacity)...)
		bp.M_moveCapacity *= 2
	}

	bp.M_moveBuffer[bp.M_moveCount] = proxyId
	bp.M_moveCount++
}

func (bp *BroadPhase) UnBufferMove(proxyId int) {
	for i := 0; i < bp.M_moveCount; i++ {
		if bp.M_moveBuffer[i] == proxyId {
			bp.M_moveBuffer[i] = E_nullProxy
		}
	}
}

// This is called from b2DynamicTree::Query when we are gathering pairs.
func (bp *BroadPhase) QueryCallback(proxyId int) bool {

	// A proxy cannot form a pair with itself.
	if proxyId == bp.M_queryProxyId {
		return true
	}

	moved := bp.M_tree.WasMoved(proxyId)
	if moved && proxyId > bp.M_queryProxyId {
		// Both proxies are moving. Avoid duplicate pairs.
		return true
	}

	// Grow the pair buffer as needed.
	if bp.M_pairCount == bp.M_pairCapacity {
		bp.M_pairBuffer = append(bp.M_pairBuffer, make([]Pair, bp.M_pairCapacity)...)
		bp.M_pairCapacity = bp.M_pairCapacity + (bp.M_pairCapacity >> 1)
	}

	bp.M_pairBuffer[bp.M_pairCount].ProxyIdA = min(proxyId, bp.M_queryProxyId)
	bp.M_pairBuffer[bp.M_pairCount].ProxyIdB = max(proxyId, bp.M_queryProxyId)
	bp.M_pairCount++

	return true
}

func (bp *BroadPhase) Query(callback TreeQueryCallback, aabb AABB) {
	bp.M_tree.Query(callback, aabb)
}

func (bp *BroadPhase) RayCast(callback TreeRayCastCallback, input RayCastInput) {
	bp.M_tree.RayCast(callback, input)
}

func (bp *BroadPhase) ShiftOrigin(newOrigin Vec2) {
	bp.M_tree.ShiftOrigin(newOrigin)
}
