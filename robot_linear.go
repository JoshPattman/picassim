package main

import (
	"time"

	"github.com/JoshPattman/jcode"
	"github.com/gopxl/pixel"
	"github.com/gopxl/pixel/imdraw"
	"golang.org/x/image/colornames"
)

type LinearRobotConfig struct {
	MaxAcceleration        float64 `json:"max_acceleration"`
	HomingFactor           float64 `json:"homing_factor"`
	AccelerationMultiplier float64 `json:"acceleration_multiplier"`
}

func (r *LinearRobotConfig) BuildRobot(start jcode.Waypoint) Robot {
	return &LinearRobot{
		lastPosition:           pixel.V(start.XPos, start.YPos),
		maxAcceleration:        r.MaxAcceleration,
		homingFactor:           r.HomingFactor,
		lastUpdate:             time.Now(),
		accelerationMultiplier: r.AccelerationMultiplier,
	}
}

var _ Robot = &LinearRobot{}

type LinearRobot struct {
	lastUpdate             time.Time
	lastPosition           pixel.Vec
	currentVelocity        pixel.Vec
	maxAcceleration        float64
	accelerationMultiplier float64
	homingFactor           float64
}

// Draw implements Robot.
func (a *LinearRobot) Draw(with *imdraw.IMDraw) {
	with.Color = colornames.Blue
	with.Push(a.lastPosition)
	with.Circle(0.5, 0)
}

// Update implements Robot.
func (a *LinearRobot) Update(target, targetVel pixel.Vec) {
	now := time.Now()
	dt := now.Sub(a.lastUpdate).Seconds()
	delta := target.Sub(a.lastPosition)
	targetCorrectVel := delta.Scaled(a.homingFactor)
	targetVel = targetVel.Add(targetCorrectVel)
	accel := targetVel.Sub(a.currentVelocity).Scaled(a.accelerationMultiplier)
	if accel.Len() > a.maxAcceleration {
		accel = accel.Unit().Scaled(a.maxAcceleration)
	}
	a.currentVelocity = a.currentVelocity.Add(accel.Scaled(dt))
	a.lastPosition = a.lastPosition.Add(a.currentVelocity.Scaled(dt))
	a.lastUpdate = now
}

func (a *LinearRobot) EEPos() pixel.Vec {
	return a.lastPosition
}
