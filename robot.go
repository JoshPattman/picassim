package main

import (
	"math"
	"time"

	"github.com/JoshPattman/jcode"
	"github.com/gopxl/pixel"
	"github.com/gopxl/pixel/imdraw"
	"golang.org/x/image/colornames"
)

type Robot interface {
	Key() string
	Update(target, targetVel pixel.Vec)
	Draw(with *imdraw.IMDraw)
	EEPos() pixel.Vec
}

var _ Robot = &AccelerationRobot{}

type AccelerationRobot struct {
	lastUpdate             time.Time
	lastPosition           pixel.Vec
	currentVelocity        pixel.Vec
	maxAcceleration        float64
	accelerationMultiplier float64
	homingFactor           float64
}

func NewAccelerationRobot(maxAccel, homingFactor, accelerationMultiplier float64, start jcode.Waypoint) *AccelerationRobot {
	return &AccelerationRobot{
		lastPosition:           pixel.V(start.XPos, start.YPos),
		maxAcceleration:        maxAccel,
		homingFactor:           homingFactor,
		lastUpdate:             time.Now(),
		accelerationMultiplier: accelerationMultiplier,
	}
}

// Draw implements Robot.
func (a *AccelerationRobot) Draw(with *imdraw.IMDraw) {
	with.Color = colornames.Blue
	with.Push(a.lastPosition)
	with.Circle(0.5, 0)
}

// Key implements Robot.
func (a *AccelerationRobot) Key() string {
	return "accel"
}

// Update implements Robot.
func (a *AccelerationRobot) Update(target, targetVel pixel.Vec) {
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

func (a *AccelerationRobot) EEPos() pixel.Vec {
	return a.lastPosition
}

var _ Robot = &PicassoRobot{}

type PicassoRobot struct {
	leftAngle, rightAngle  float64
	leftSpeed, rightSpeed  float64
	maxAcceleration        float64
	maxSpeed               float64
	limbLength             float64
	spread                 float64
	lastUpdate             time.Time
	accelerationMultiplier float64
	homingFactor           float64
}

func NewPicassoRobot(limbLength, spread, accelerationMult, homingFactor, maxAccel, maxSpeed float64) *PicassoRobot {
	return &PicassoRobot{
		limbLength:             limbLength,
		spread:                 spread,
		leftAngle:              math.Pi / 4,
		rightAngle:             -math.Pi / 4,
		lastUpdate:             time.Now(),
		accelerationMultiplier: accelerationMult,
		homingFactor:           homingFactor,
		maxAcceleration:        maxAccel,
		maxSpeed:               maxSpeed,
	}
}

func (p *PicassoRobot) joints() (pixel.Vec, pixel.Vec, pixel.Vec, pixel.Vec, pixel.Vec) {
	sv := pixel.V(p.spread/2, 0)
	left := pixel.V(0, 1).Rotated(p.leftAngle).Scaled(p.limbLength).Sub(sv)
	right := pixel.V(0, 1).Rotated(p.rightAngle).Scaled(p.limbLength).Add(sv)
	mp := left.Add(right).Scaled(0.5)
	dtmp := left.Sub(mp).Len()
	hamp := math.Sqrt(p.limbLength*p.limbLength - dtmp*dtmp)
	ep := mp.Add(left.Sub(right).Rotated(math.Pi / -2).Unit().Scaled(hamp))

	return sv.Scaled(-1), sv, left, right, ep
}

// Draw implements Robot.
func (p *PicassoRobot) Draw(with *imdraw.IMDraw) {
	rootL, rootR, jointL, jointR, ep := p.joints()
	with.Color = colornames.Blue
	with.Push(rootL, jointL)
	with.Line(0.1)
	with.Push(rootR, jointR)
	with.Line(0.1)
	with.Push(ep, jointL)
	with.Line(0.1)
	with.Push(ep, jointR)
	with.Line(0.1)
}

// EEPos implements Robot.
func (p *PicassoRobot) EEPos() pixel.Vec {
	_, _, _, _, ep := p.joints()
	return ep
}

// Key implements Robot.
func (p *PicassoRobot) Key() string {
	return "sdvibdivdf"
}

func cosineAngle(a, b, c float64) (A float64) {
	return math.Acos((b*b + c*c - a*a) / (2 * b * c))
}

func (p *PicassoRobot) ik(to pixel.Vec) (float64, float64) {
	lRoot, rRoot, _, _, _ := p.joints()
	lDist := lRoot.Sub(to).Len()
	rDist := rRoot.Sub(to).Len()
	lTotalAngle := cosineAngle(p.limbLength, lDist, p.limbLength)
	bTotalAngle := -cosineAngle(p.limbLength, rDist, p.limbLength)
	leftAngle := lTotalAngle - (math.Pi/2 - lRoot.To(to).Angle())
	rightAngle := bTotalAngle - (math.Pi/2 - rRoot.To(to).Angle())
	return leftAngle, rightAngle
}

func (p *PicassoRobot) ikv(at pixel.Vec, vel pixel.Vec) (float64, float64) {
	atL, atR := p.ik(at)
	epsilon := 0.02
	atLE, atRE := p.ik(at.Add(vel.Scaled(epsilon)))
	return (atLE - atL) / epsilon, (atRE - atR) / epsilon
}

func clamp(x, mi, ma float64) float64 {
	if x < mi {
		return mi
	} else if x > ma {
		return ma
	} else {
		return x
	}
}

func clamps(x, b float64) float64 {
	return clamp(x, -b, b)
}

// Update implements Robot.
func (p *PicassoRobot) Update(target pixel.Vec, targetVel pixel.Vec) {
	// Time stuff
	now := time.Now()
	dt := now.Sub(p.lastUpdate).Seconds()

	// Corrective velocity
	pos := p.EEPos()
	delta := target.Sub(pos)
	targetCorrectVel := delta.Scaled(p.homingFactor)
	targetVel = targetVel.Add(targetCorrectVel)

	tVelL, tVelR := p.ikv(pos, targetVel)
	if math.IsNaN(tVelL) || math.IsNaN(tVelR) {
		tVelL, tVelR = 0, 0
	}

	accL := clamps((tVelL-p.leftSpeed)*p.accelerationMultiplier, p.maxAcceleration)
	accR := clamps((tVelR-p.rightSpeed)*p.accelerationMultiplier, p.maxAcceleration)

	p.leftSpeed += accL * dt
	p.rightSpeed += accR * dt

	p.leftSpeed = clamps(p.leftSpeed, p.maxSpeed)
	p.rightSpeed = clamps(p.rightSpeed, p.maxSpeed)

	p.leftAngle += p.leftSpeed * dt
	p.rightAngle += p.rightSpeed * dt

	p.lastUpdate = now
}

/*now := time.Now()
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
a.lastUpdate = now*/
