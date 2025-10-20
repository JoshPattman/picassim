package main

import (
	"errors"
	"time"

	"github.com/JoshPattman/jcode"
	"github.com/gopxl/pixel"
)

type TargetStateMachine struct {
	Last                 jcode.Waypoint
	Next                 jcode.Waypoint
	LastTime             time.Time
	NextDuration         time.Duration
	Speed                jcode.Speed
	CurrentTime          time.Time
	PenMode              jcode.PenMode
	HasActiveInstruction bool
}

func (sm *TargetStateMachine) Reset(to jcode.Waypoint) {
	sm.CurrentTime = time.Now()
	sm.LastTime = time.Now()
	sm.NextDuration = 0
	sm.Speed.Speed = 0
	sm.Last = to
	sm.Next = to
	sm.PenMode = jcode.PenUp
	sm.HasActiveInstruction = false
}

func (sm *TargetStateMachine) Update(comms *jcode.RobotCommunicator) error {
	sm.CurrentTime = time.Now()
	needNextInstruction := false
	doneIns := func() error {
		sm.HasActiveInstruction = false
		select {
		case comms.ToController() <- jcode.Consumed{}:
			return nil
		case err := <-comms.Error():
			return err
		}
	}
	if sm.CurrentTime.Sub(sm.LastTime) >= sm.NextDuration || sm.NextDuration == 0 {
		if sm.HasActiveInstruction {
			err := doneIns()
			if err != nil {
				return err
			}
		}
		needNextInstruction = true
		sm.Last = sm.Next
		sm.LastTime = sm.CurrentTime
		sm.NextDuration = 0

	}
	for needNextInstruction {
		var ins jcode.Instruction
		noMoreInstructions := false
		select {
		case ins = <-comms.FromController():
		case err := <-comms.Error():
			return err
		default:
			noMoreInstructions = false
		}
		if noMoreInstructions {
			break
		}
		sm.HasActiveInstruction = true
		switch ins := ins.(type) {
		case jcode.Waypoint:
			if !jcode.Possible(sm.Last, ins, sm.Speed) {
				select {
				case comms.ToController() <- jcode.Log{Message: "Error: Cannot move to a non zero waypoint if speed is 0"}:
					return errors.New("cannot move to a non zero waypoint if speed is 0")
				case err := <-comms.Error():
					return err
				}
			}
			sm.Next = ins
			sm.NextDuration = jcode.Time(sm.Last, sm.Next, sm.Speed)
			needNextInstruction = false
		case jcode.Delay:
			sm.Next = sm.Last
			sm.NextDuration = ins.Duration
			needNextInstruction = false
		case jcode.AutoHome:
			// Its a bit bad we modify the speed here but oh well
			sm.Speed = jcode.Speed{Speed: 1}
			sm.Next = jcode.Waypoint{XPos: 0, YPos: 10}
			sm.NextDuration = jcode.Time(sm.Last, sm.Next, sm.Speed)
		case jcode.Speed:
			sm.Speed = ins
			err := doneIns()
			if err != nil {
				return err
			}
		case jcode.Pen:
			sm.PenMode = ins.Mode
			err := doneIns()
			if err != nil {
				return err
			}
		}
	}
	return nil
}

func (sm *TargetStateMachine) TargetPosition() jcode.Waypoint {
	if sm.NextDuration == 0 {
		return sm.Last
	}
	dt := sm.NextDuration.Seconds()
	t := sm.CurrentTime.Sub(sm.LastTime).Seconds() / dt
	return jcode.Waypoint{
		XPos: lerp(sm.Last.XPos, sm.Next.XPos, t),
		YPos: lerp(sm.Last.YPos, sm.Next.YPos, t),
	}
}

func (sm *TargetStateMachine) TargetVelocity() jcode.Waypoint {
	if sm.NextDuration == 0 || sm.CurrentTime.After(sm.LastTime.Add(sm.NextDuration)) || sm.Last == sm.Next {
		return jcode.Waypoint{}
	}
	v := pixel.V(sm.Next.XPos, sm.Next.YPos).Sub(pixel.V(sm.Last.XPos, sm.Last.YPos)).Unit().Scaled(sm.Speed.Speed)
	return jcode.Waypoint{
		XPos: v.X,
		YPos: v.Y,
	}
}

func lerp(x, y, t float64) float64 {
	return x*(1-t) + y*t
}
