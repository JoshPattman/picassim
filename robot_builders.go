package main

import (
	"encoding/json"
	"fmt"
	"math"

	"github.com/JoshPattman/jcode"
)

// Keeps track of all possible robot builders
type RobotIndex map[string]RobotBuilder

var DefaultRobotIndex = RobotIndex{
	"picasso": &RobotConfigPicasso{
		LimbLength:             8,
		Spread:                 2,
		MaxAcceleration:        0.5 * 2 * math.Pi,
		MaxSpeed:               2 * math.Pi,
		AccelerationMultiplier: 20,
		HomingFactor:           2,
	},
	"basic-body2d": &RobotConfigAccel{
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
		case "twoarm":
			builder = &RobotConfigPicasso{}
		case "body2d":
			builder = &RobotConfigAccel{}
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
		case *RobotConfigPicasso:
			rci.RobotType = "twoarm"
		case *RobotConfigAccel:
			rci.RobotType = "body2d"
		default:
			return nil, fmt.Errorf("invalid robot type %T", builder)
		}
		intermediateDict[key] = rci
	}
	return json.Marshal(intermediateDict)
}

type RobotConfigAccel struct {
	MaxAcceleration        float64 `json:"max_acceleration"`
	HomingFactor           float64 `json:"homing_factor"`
	AccelerationMultiplier float64 `json:"acceleration_multiplier"`
}

func (r *RobotConfigAccel) BuildRobot(start jcode.Waypoint) Robot {
	return NewAccelerationRobot(r.MaxAcceleration, r.HomingFactor, r.AccelerationMultiplier, start)
}

type RobotConfigPicasso struct {
	LimbLength             float64 `json:"limb_length"`
	Spread                 float64 `json:"spread"`
	MaxAcceleration        float64 `json:"max_acceleration"`
	HomingFactor           float64 `json:"homing_factor"`
	AccelerationMultiplier float64 `json:"acceleration_multiplier"`
	MaxSpeed               float64 `json:"max_speed"`
}

func (r *RobotConfigPicasso) BuildRobot(start jcode.Waypoint) Robot {
	return NewPicassoRobot(r.LimbLength, r.Spread, r.AccelerationMultiplier, r.HomingFactor, r.MaxAcceleration, r.MaxSpeed)
}
