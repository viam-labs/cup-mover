package multiposesexecutionswitch

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var Model = resource.NewModel("viam-labs", "cup-mover", "multi-poses-execution-switch")

// Mode selects how a waypoint is interpreted and executed.
const (
	// ModePose moves a frame to an absolute Cartesian pose via the Motion service.
	ModePose = "pose"
	// ModeJoint commands the arm's joints directly via MoveToJointPositions.
	ModeJoint = "joint"
)

func init() {
	resource.RegisterComponent(
		toggleswitch.API,
		Model,
		resource.Registration[toggleswitch.Switch, *Config]{
			Constructor: newMultiPosesExecutionSwitch,
		},
	)
}

type Config struct {
	// Mode is "pose" (default) or "joint". In pose mode, waypoints are absolute
	// Cartesian poses executed via the Motion service. In joint mode, waypoints
	// are joint-position arrays executed directly on the arm.
	Mode string `json:"mode,omitempty"`

	// ComponentName is the frame to move in pose mode (e.g. "gripper" or "arm"),
	// or the arm component to command in joint mode.
	ComponentName string `json:"component_name"`

	// ReferenceFrame is the frame the poses are expressed in (pose mode only).
	ReferenceFrame string `json:"reference_frame,omitempty"`

	// Motion is the motion service to use (pose mode only).
	Motion string `json:"motion,omitempty"`

	// Waypoints are the saved positions exposed as switch positions.
	Waypoints []Waypoint `json:"waypoints"`
}

// Waypoint is one saved position. In pose mode the X/Y/Z + orientation fields
// are used; in joint mode the Joints array (radians) is used.
type Waypoint struct {
	Name string `json:"name"`

	// Pose-mode fields.
	X     float64 `json:"x,omitempty"`
	Y     float64 `json:"y,omitempty"`
	Z     float64 `json:"z,omitempty"`
	OX    float64 `json:"o_x,omitempty"`
	OY    float64 `json:"o_y,omitempty"`
	OZ    float64 `json:"o_z,omitempty"`
	Theta float64 `json:"theta,omitempty"`

	// Joint-mode field: one value per joint, in radians.
	Joints []float64 `json:"joints,omitempty"`
}

func (cfg *Config) mode() string {
	if cfg.Mode == "" {
		return ModePose
	}
	return cfg.Mode
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.ComponentName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "component_name")
	}

	mode := cfg.mode()
	switch mode {
	case ModePose:
		if cfg.Motion == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "motion")
		}
	case ModeJoint:
		// motion is unused in joint mode.
	default:
		return nil, nil, fmt.Errorf("%s: unknown mode %q (supported: %q, %q)", path, mode, ModePose, ModeJoint)
	}

	if len(cfg.Waypoints) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "waypoints")
	}

	seen := make(map[string]bool, len(cfg.Waypoints))
	for i, w := range cfg.Waypoints {
		if w.Name == "" {
			return nil, nil, fmt.Errorf("%s: waypoints[%d] is missing required field \"name\"", path, i)
		}
		if seen[w.Name] {
			return nil, nil, fmt.Errorf("%s: waypoints[%d] has duplicate name %q", path, i, w.Name)
		}
		seen[w.Name] = true
		if mode == ModeJoint && len(w.Joints) == 0 {
			return nil, nil, fmt.Errorf("%s: waypoints[%d] (%q) is missing required field \"joints\" in joint mode", path, i, w.Name)
		}
	}

	switch mode {
	case ModePose:
		deps := []string{cfg.ComponentName}
		if cfg.Motion == "builtin" {
			deps = append(deps, motion.Named("builtin").String())
		} else {
			deps = append(deps, cfg.Motion)
		}
		return deps, nil, nil
	default: // ModeJoint
		return []string{arm.Named(cfg.ComponentName).String()}, nil, nil
	}
}

type multiPosesExecutionSwitch struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name      resource.Name
	logger    logging.Logger
	cfg       *Config
	mode      string
	motion    motion.Service // pose mode
	arm       arm.Arm        // joint mode
	poseNames []string

	mu        sync.Mutex
	position  uint32
	executing atomic.Bool
}

func newMultiPosesExecutionSwitch(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (toggleswitch.Switch, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	if conf.ReferenceFrame == "" {
		conf.ReferenceFrame = referenceframe.World
	}

	s := &multiPosesExecutionSwitch{
		name:   rawConf.ResourceName(),
		logger: logger,
		cfg:    conf,
		mode:   conf.mode(),
	}

	switch s.mode {
	case ModePose:
		s.motion, err = motion.FromProvider(deps, conf.Motion)
		if err != nil {
			return nil, err
		}
	case ModeJoint:
		s.arm, err = arm.FromProvider(deps, conf.ComponentName)
		if err != nil {
			return nil, fmt.Errorf("getting arm %q: %w", conf.ComponentName, err)
		}
	}

	s.poseNames = make([]string, len(conf.Waypoints))
	for i, w := range conf.Waypoints {
		s.poseNames[i] = w.Name
	}

	return s, nil
}

func (s *multiPosesExecutionSwitch) Name() resource.Name {
	return s.name
}

func (s *multiPosesExecutionSwitch) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *multiPosesExecutionSwitch) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if name, ok := cmd["set_position_by_name"].(string); ok {
		for i, pn := range s.poseNames {
			if pn == name {
				if err := s.SetPosition(ctx, uint32(i), nil); err != nil {
					return nil, err
				}
				return nil, nil
			}
		}
		return nil, fmt.Errorf("unknown waypoint name %q", name)
	}

	if _, ok := cmd["get_current_position_name"]; ok {
		s.mu.Lock()
		pos := s.position
		s.mu.Unlock()
		return map[string]interface{}{"position_name": s.poseNames[pos]}, nil
	}

	return nil, fmt.Errorf("unknown command, supported: set_position_by_name, get_current_position_name")
}

func (s *multiPosesExecutionSwitch) GetNumberOfPositions(ctx context.Context, extra map[string]interface{}) (uint32, []string, error) {
	return uint32(len(s.poseNames)), s.poseNames, nil
}

func (s *multiPosesExecutionSwitch) GetPosition(ctx context.Context, extra map[string]interface{}) (uint32, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	return s.position, nil
}

func (s *multiPosesExecutionSwitch) SetPosition(ctx context.Context, position uint32, extra map[string]interface{}) error {
	if position >= uint32(len(s.cfg.Waypoints)) {
		return fmt.Errorf("requested position %d is out of range (max %d)", position, len(s.cfg.Waypoints)-1)
	}
	if !s.executing.CompareAndSwap(false, true) {
		return errors.New("switch is currently executing")
	}
	defer s.executing.Store(false)

	w := s.cfg.Waypoints[position]

	var err error
	switch s.mode {
	case ModePose:
		err = s.moveToPose(ctx, w, position)
	case ModeJoint:
		err = s.moveToJoints(ctx, w, position)
	}
	if err != nil {
		return err
	}

	s.mu.Lock()
	s.position = position
	s.mu.Unlock()
	return nil
}

func (s *multiPosesExecutionSwitch) moveToPose(ctx context.Context, w Waypoint, position uint32) error {
	s.logger.Infof("moving %s to pose %q (index %d)", s.cfg.ComponentName, w.Name, position)

	pose := spatialmath.NewPose(
		r3.Vector{X: w.X, Y: w.Y, Z: w.Z},
		&spatialmath.OrientationVectorDegrees{OX: w.OX, OY: w.OY, OZ: w.OZ, Theta: w.Theta},
	)
	destination := referenceframe.NewPoseInFrame(s.cfg.ReferenceFrame, pose)

	if _, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.cfg.ComponentName,
		Destination:   destination,
	}); err != nil {
		return fmt.Errorf("failed to move to pose %q: %w", w.Name, err)
	}
	return nil
}

func (s *multiPosesExecutionSwitch) moveToJoints(ctx context.Context, w Waypoint, position uint32) error {
	s.logger.Infof("moving %s to joints %q (index %d): %v", s.cfg.ComponentName, w.Name, position, w.Joints)

	// referenceframe.Input is a float64 alias (radians for revolute joints), so
	// the configured Joints slice is used directly.
	inputs := make([]referenceframe.Input, len(w.Joints))
	copy(inputs, w.Joints)

	if err := s.arm.MoveToJointPositions(ctx, inputs, nil); err != nil {
		return fmt.Errorf("failed to move to joints %q: %w", w.Name, err)
	}
	return nil
}
