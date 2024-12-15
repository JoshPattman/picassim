package main

import (
	"encoding/json"
	"fmt"
	"math"

	"github.com/JoshPattman/jcode"
	"github.com/gopxl/pixel"
	"github.com/gopxl/pixel/imdraw"
)

type Robot interface {
	Update(target, targetVel pixel.Vec)
	Draw(with *imdraw.IMDraw)
	EEPos() pixel.Vec
}

// Keeps track of all possible robot builders
type RobotIndex map[string]RobotBuilder

var DefaultRobotIndex = RobotIndex{
	"picasso": &TwoArmRobotConfig{
		LimbInnerLength:        7,
		LimbOuterLength:        8,
		Spread:                 2,
		MaxAcceleration:        0.5 * 2 * math.Pi,
		MaxSpeed:               2 * math.Pi,
		AccelerationMultiplier: 20,
		HomingFactor:           2,
		MinMotorAngle:          0,
		MaxMotorAngle:          math.Pi / 2,
	},
	"basic-body2d": &LinearRobotConfig{
		MaxAcceleration:        20,
		HomingFactor:           2,
		AccelerationMultiplier: 20,
	},
}

type RobotBuilder interface {
	BuildRobot(start jcode.Waypoint) Robot
}

var _ json.Marshaler = RobotIndex{}
var _ json.Unmarshaler = RobotIndex{}

const (
	twoArmRobotType = "twoarm"
	linearRobotType = "linear"
)

type robotConfigIntermediate struct {
	RobotType string `json:"robot_type"`
	RobotData any    `json:"robot_data"`
}

// UnmarshalJSON implements json.Unmarshaler.
func (r RobotIndex) UnmarshalJSON(d []byte) error {
	intermediateDict := make(map[string]robotConfigIntermediate)
	if err := json.Unmarshal(d, &intermediateDict); err != nil {
		return err
	}
	for key, rci := range intermediateDict {
		builderBytes, err := json.Marshal(rci.RobotData)
		if err != nil {
			return err
		}
		var builder RobotBuilder
		switch rci.RobotType {
		case twoArmRobotType:
			builder = &TwoArmRobotConfig{}
		case linearRobotType:
			builder = &LinearRobotConfig{}
		default:
			return fmt.Errorf("invalid robot type %s", rci.RobotType)
		}
		err = json.Unmarshal(builderBytes, builder)
		if err != nil {
			return err
		}
		r[key] = builder
	}
	return nil
}

// MarshalJSON implements json.Marshaler.
func (r RobotIndex) MarshalJSON() ([]byte, error) {
	intermediateDict := make(map[string]robotConfigIntermediate)
	for key, builder := range r {
		rci := robotConfigIntermediate{
			RobotData: builder,
		}
		switch builder.(type) {
		case *TwoArmRobotConfig:
			rci.RobotType = twoArmRobotType
		case *LinearRobotConfig:
			rci.RobotType = linearRobotType
		default:
			return nil, fmt.Errorf("invalid robot type %T", builder)
		}
		intermediateDict[key] = rci
	}
	return json.Marshal(intermediateDict)
}
