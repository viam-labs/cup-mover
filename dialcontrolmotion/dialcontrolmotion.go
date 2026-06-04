// Package dialcontrolmotion registers a viam-labs:cup-mover:dial-control-motion
// generic service. It nudges an arm by configurable steps, either driven by
// absolute dial positions (Stream Deck-style) or by direct jog commands.
package dialcontrolmotion

import (
	"context"
	"fmt"
	"math"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var Model = resource.NewModel("viam-labs", "cup-mover", "dial-control-motion")

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newDialControlMotion,
		},
	)
}

type Config struct {
	ArmName               string  `json:"arm_name"`
	FrameName             string  `json:"frame_name,omitempty"`
	MotionServiceName     string  `json:"motion_service_name,omitempty"`
	DialMoveXMM           float64 `json:"dial_move_x_mm,omitempty"`
	DialMoveYMM           float64 `json:"dial_move_y_mm,omitempty"`
	DialMoveZMM           float64 `json:"dial_move_z_mm,omitempty"`
	DialMoveOrientationMM float64 `json:"dial_move_orientation_mm,omitempty"`
	DialMaxPosition       float64 `json:"dial_max_position,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.ArmName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm_name")
	}
	deps := []string{arm.Named(cfg.ArmName).String()}
	deps = append(deps, motion.Named(cfg.motionServiceName()).String())
	return deps, nil, nil
}

func (cfg *Config) motionServiceName() string {
	if cfg.MotionServiceName == "" {
		return "builtin"
	}
	return cfg.MotionServiceName
}

func (cfg *Config) frameName() string {
	if cfg.FrameName == "" {
		return cfg.ArmName
	}
	return cfg.FrameName
}

type dialControlMotion struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name      resource.Name
	logger    logging.Logger
	cfg       *Config
	arm       arm.Arm
	motion    motion.Service
	frameName string

	mu sync.Mutex
	// Last absolute dial positions and inferred directions (Stream Deck flow).
	lastDialX              *float64
	lastDialY              *float64
	lastDialZ              *float64
	lastDialOrientation    *float64
	lastDialDirX           float64
	lastDialDirY           float64
	lastDialDirZ           float64
	lastDialDirOrientation float64
}

func newDialControlMotion(_ context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	armComp, err := arm.FromProvider(deps, conf.ArmName)
	if err != nil {
		return nil, fmt.Errorf("arm %q not found in dependencies: %w", conf.ArmName, err)
	}

	motionSvc, err := motion.FromProvider(deps, conf.motionServiceName())
	if err != nil {
		return nil, fmt.Errorf("motion service %q not found in dependencies: %w", conf.motionServiceName(), err)
	}

	return &dialControlMotion{
		name:      rawConf.ResourceName(),
		logger:    logger,
		cfg:       conf,
		arm:       armComp,
		motion:    motionSvc,
		frameName: conf.frameName(),
	}, nil
}

func (s *dialControlMotion) Name() resource.Name {
	return s.name
}

func (s *dialControlMotion) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *dialControlMotion) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	// Direct jogs — useful from a CLI / Control tab without a Stream Deck.
	if v, ok := cmd["jog_x"]; ok {
		return s.jog(ctx, "x", v)
	}
	if v, ok := cmd["jog_y"]; ok {
		return s.jog(ctx, "y", v)
	}
	if v, ok := cmd["jog_z"]; ok {
		return s.jog(ctx, "z", v)
	}
	if v, ok := cmd["jog_orientation"]; ok {
		return s.jog(ctx, "orientation", v)
	}

	// Stream Deck-style dial input — service infers direction from delta.
	if v, ok := cmd["dial_move_x"]; ok {
		return s.handleDialMove(ctx, "x", v)
	}
	if v, ok := cmd["dial_move_y"]; ok {
		return s.handleDialMove(ctx, "y", v)
	}
	if v, ok := cmd["dial_move_z"]; ok {
		return s.handleDialMove(ctx, "z", v)
	}
	if v, ok := cmd["dial_move_orientation"]; ok {
		return s.handleDialMove(ctx, "orientation", v)
	}

	if _, ok := cmd["get_pose"]; ok {
		return s.getPose(ctx)
	}

	if _, ok := cmd["get_joints"]; ok {
		return s.getJoints(ctx)
	}

	if v, ok := cmd["jog_joint"]; ok {
		return s.jogJoint(ctx, v, cmd["by"])
	}

	return nil, fmt.Errorf("unknown command, supported: jog_{x,y,z,orientation}, dial_move_{x,y,z,orientation}, get_pose, get_joints, jog_joint")
}

// jog moves the arm by mm along the given axis. mm may be negative.
func (s *dialControlMotion) jog(ctx context.Context, axis string, mmVal interface{}) (map[string]interface{}, error) {
	mm, ok := toFloat64(mmVal)
	if !ok {
		return nil, fmt.Errorf("jog_%s: invalid value %v (expected number)", axis, mmVal)
	}
	return s.handleMoveArm(ctx, axis, mm)
}

// getPose returns the current pose of the configured frame (arm flange by
// default, or whatever was set via frame_name — typically "gripper") expressed
// in the world frame. Output shape matches multi-poses-execution-switch's
// pose entries.
func (s *dialControlMotion) getPose(ctx context.Context) (map[string]interface{}, error) {
	pif, err := s.motion.GetPose(ctx, s.frameName, referenceframe.World, nil, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get pose for frame %q: %w", s.frameName, err)
	}
	pose := pif.Pose()
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	return map[string]interface{}{
		"x":     pt.X,
		"y":     pt.Y,
		"z":     pt.Z,
		"o_x":   ov.OX,
		"o_y":   ov.OY,
		"o_z":   ov.OZ,
		"theta": ov.Theta,
	}, nil
}

// getJoints returns the arm's current joint positions in radians. Output shape
// matches a multi-poses-execution-switch joint-mode waypoint ("joints").
func (s *dialControlMotion) getJoints(ctx context.Context) (map[string]interface{}, error) {
	inputs, err := s.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get joint positions: %w", err)
	}
	// referenceframe.Input is a float64 alias (radians for revolute joints).
	joints := make([]float64, len(inputs))
	copy(joints, inputs)
	return map[string]interface{}{"joints": joints}, nil
}

// jogJoint nudges a single joint by delta radians (delta may be negative),
// leaving all other joints where they are.
func (s *dialControlMotion) jogJoint(ctx context.Context, indexVal, byVal interface{}) (map[string]interface{}, error) {
	idxF, ok := toFloat64(indexVal)
	if !ok {
		return nil, fmt.Errorf("jog_joint: invalid joint index %v (expected number)", indexVal)
	}
	idx := int(idxF)
	delta, ok := toFloat64(byVal)
	if !ok {
		return nil, fmt.Errorf("jog_joint: invalid or missing \"by\" value %v (expected radians)", byVal)
	}

	inputs, err := s.arm.JointPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get joint positions: %w", err)
	}
	if idx < 0 || idx >= len(inputs) {
		return nil, fmt.Errorf("jog_joint: index %d out of range (arm has %d joints)", idx, len(inputs))
	}
	inputs[idx] += delta

	if err := s.arm.MoveToJointPositions(ctx, inputs, nil); err != nil {
		return nil, fmt.Errorf("failed to move joints: %w", err)
	}

	joints := make([]float64, len(inputs))
	copy(joints, inputs)
	return map[string]interface{}{"status": "moved", "joint": idx, "by": delta, "joints": joints}, nil
}

func (s *dialControlMotion) handleMoveArm(ctx context.Context, axis string, mm float64) (map[string]interface{}, error) {
	currentPose, err := s.arm.EndPosition(ctx, map[string]interface{}{})
	if err != nil {
		return nil, fmt.Errorf("failed to get current arm position: %w", err)
	}
	pt := currentPose.Point()
	switch axis {
	case "x":
		pt.X += mm
	case "y":
		pt.Y += mm
	case "z":
		pt.Z += mm
	case "orientation":
		ov := currentPose.Orientation().OrientationVectorDegrees()
		norm := math.Sqrt(ov.OX*ov.OX + ov.OY*ov.OY + ov.OZ*ov.OZ)
		if norm > 0 {
			pt.X += mm * ov.OX / norm
			pt.Y += mm * ov.OY / norm
			pt.Z += mm * ov.OZ / norm
		}
	default:
		return nil, fmt.Errorf("unknown axis %q", axis)
	}
	newPose := spatialmath.NewPose(pt, currentPose.Orientation())
	if err := s.arm.MoveToPosition(ctx, newPose, map[string]interface{}{}); err != nil {
		return nil, fmt.Errorf("failed to move arm: %w", err)
	}
	return map[string]interface{}{"status": "moved", "axis": axis, "mm": mm}, nil
}

func (s *dialControlMotion) handleDialMove(ctx context.Context, axis string, dialValue interface{}) (map[string]interface{}, error) {
	var mm float64
	switch axis {
	case "x":
		mm = s.cfg.DialMoveXMM
	case "y":
		mm = s.cfg.DialMoveYMM
	case "z":
		mm = s.cfg.DialMoveZMM
	case "orientation":
		mm = s.cfg.DialMoveOrientationMM
	}
	if mm == 0 {
		mm = 1
	}

	dialVal, ok := toFloat64(dialValue)
	if !ok {
		return nil, fmt.Errorf("dial_move_%s: invalid value %v", axis, dialValue)
	}

	s.mu.Lock()
	var last **float64
	var lastDir *float64
	switch axis {
	case "x":
		last, lastDir = &s.lastDialX, &s.lastDialDirX
	case "y":
		last, lastDir = &s.lastDialY, &s.lastDialDirY
	case "z":
		last, lastDir = &s.lastDialZ, &s.lastDialDirZ
	case "orientation":
		last, lastDir = &s.lastDialOrientation, &s.lastDialDirOrientation
	}
	if *last == nil {
		// First reading — store position and skip move (no direction yet).
		*last = &dialVal
		s.mu.Unlock()
		return map[string]interface{}{"status": "dial_initialized", "axis": axis, "position": dialVal}, nil
	}
	maxPos := s.cfg.DialMaxPosition
	if maxPos == 0 {
		maxPos = 100
	}
	delta := dialVal - **last
	// Correct for rollover: a jump > half the range means it wrapped.
	if delta > maxPos/2 {
		delta -= maxPos + 1
	} else if delta < -maxPos/2 {
		delta += maxPos + 1
	}
	var direction float64
	if **last == 0 && *lastDir != 0 {
		// At the zero boundary the dial bounces — keep the last direction.
		direction = *lastDir
	} else if delta < 0 {
		direction = -1
	} else {
		direction = 1
	}
	if direction < 0 {
		mm = -mm
	}
	*lastDir = direction
	**last = dialVal
	s.mu.Unlock()

	return s.handleMoveArm(ctx, axis, mm)
}

func toFloat64(v interface{}) (float64, bool) {
	switch n := v.(type) {
	case float64:
		return n, true
	case float32:
		return float64(n), true
	case int:
		return float64(n), true
	case int64:
		return float64(n), true
	}
	return 0, false
}
