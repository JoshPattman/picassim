package main

import (
	"math"
	"time"

	"github.com/JoshPattman/jcode"
	"github.com/gopxl/pixel"
	"github.com/gopxl/pixel/imdraw"
	"golang.org/x/image/colornames"
)

type TwoArmRobotConfig struct {
	LimbInnerLength        float64 `json:"limb_inner_length"`
	LimbOuterLength        float64 `json:"limb_outer_length"`
	Spread                 float64 `json:"spread"`
	MaxAcceleration        float64 `json:"max_acceleration"`
	HomingFactor           float64 `json:"homing_factor"`
	AccelerationMultiplier float64 `json:"acceleration_multiplier"`
	MaxSpeed               float64 `json:"max_speed"`
}

func (r *TwoArmRobotConfig) BuildRobot(start jcode.Waypoint) Robot {
	return &TwoArmRobot{
		limbOuterLength:        r.LimbOuterLength,
		limbInnerLength:        r.LimbInnerLength,
		spread:                 r.Spread,
		leftAngle:              math.Pi / 4,
		rightAngle:             -math.Pi / 4,
		lastUpdate:             time.Now(),
		accelerationMultiplier: r.AccelerationMultiplier,
		homingFactor:           r.HomingFactor,
		maxAcceleration:        r.MaxAcceleration,
		maxSpeed:               r.MaxAcceleration,
	}
}

var _ Robot = &TwoArmRobot{}

type TwoArmRobot struct {
	leftAngle, rightAngle            float64
	leftSpeed, rightSpeed            float64
	maxAcceleration                  float64
	maxSpeed                         float64
	limbInnerLength, limbOuterLength float64
	spread                           float64
	lastUpdate                       time.Time
	accelerationMultiplier           float64
	homingFactor                     float64
}

// Draw implements Robot.
func (p *TwoArmRobot) Draw(with *imdraw.IMDraw) {
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
func (p *TwoArmRobot) EEPos() pixel.Vec {
	_, _, _, _, ep := p.joints()
	return ep
}

// Update implements Robot.
func (p *TwoArmRobot) Update(target pixel.Vec, targetVel pixel.Vec) {
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

func (p *TwoArmRobot) joints() (pixel.Vec, pixel.Vec, pixel.Vec, pixel.Vec, pixel.Vec) {
	sv := pixel.V(p.spread/2, 0)
	left := pixel.V(0, 1).Rotated(p.leftAngle).Scaled(p.limbInnerLength).Sub(sv)
	right := pixel.V(0, 1).Rotated(p.rightAngle).Scaled(p.limbInnerLength).Add(sv)
	mp := left.Add(right).Scaled(0.5)
	dtmp := left.Sub(mp).Len()
	hamp := math.Sqrt(p.limbOuterLength*p.limbOuterLength - dtmp*dtmp)
	ep := mp.Add(left.Sub(right).Rotated(math.Pi / -2).Unit().Scaled(hamp))

	return sv.Scaled(-1), sv, left, right, ep
}

func cosineAngle(a, b, c float64) (A float64) {
	return math.Acos((b*b + c*c - a*a) / (2 * b * c))
}

func (p *TwoArmRobot) ik(to pixel.Vec) (float64, float64) {
	lRoot, rRoot, _, _, _ := p.joints()
	lDist := lRoot.Sub(to).Len()
	rDist := rRoot.Sub(to).Len()
	lTotalAngle := cosineAngle(p.limbOuterLength, lDist, p.limbInnerLength)
	bTotalAngle := -cosineAngle(p.limbOuterLength, rDist, p.limbInnerLength)
	leftAngle := lTotalAngle - (math.Pi/2 - lRoot.To(to).Angle())
	rightAngle := bTotalAngle - (math.Pi/2 - rRoot.To(to).Angle())
	return leftAngle, rightAngle
}

func (p *TwoArmRobot) ikv(at pixel.Vec, vel pixel.Vec) (float64, float64) {
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
