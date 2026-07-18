package b2

// Adapted from https://gist.github.com/bemasher/1777766

type growableStack struct {
	top  *stackElement
	size int
}

type stackElement struct {
	value any // All types satisfy the empty interface, so we can store anything here.
	next  *stackElement
}

// Return the stack's length
func (s growableStack) Count() int {
	return s.size
}

// Push a new element onto the stack
func (s *growableStack) Push(value any) {
	s.top = &stackElement{value, s.top}
	s.size++
}

// Remove the top element from the stack and return it's value
// If the stack is empty, return nil
func (s *growableStack) Pop() (value any) {
	if s.size > 0 {
		value, s.top = s.top.value, s.top.next
		s.size--
		return
	}
	return nil
}
