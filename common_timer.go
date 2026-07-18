package b2

import "time"

// Timer for profiling. This has platform specific code and may
// not work on every platform.
type Timer struct {
	start time.Time
}

func MakeTimer() Timer {
	timer := Timer{}
	timer.Reset()
	return timer
}

func (timer *Timer) Reset() {
	timer.start = time.Now()
}

func (timer Timer) GetMilliseconds() float64 {
	return time.Since(timer.start).Seconds() * 1000
}
