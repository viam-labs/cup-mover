package multiposesexecutionswitch

import (
	"context"
	"errors"
	"fmt"
	"sync"
	"sync/atomic"

	"github.com/golang/geo/r3"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

var Model = resource.NewModel("viam-labs", "cup-mover", "multi-poses-execution-switch")

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
	ReferenceFrame string `json:"reference_frame,omitempty"`
	ComponentName  string `json:"component_name"`
	Motion         string `json:"motion"`
	Poses          []Pose `json:"poses"`
}

type Pose struct {
	Name  string  `json:"name"`
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	OX    float64 `json:"o_x"`
	OY    float64 `json:"o_y"`
	OZ    float64 `json:"o_z"`
	Theta float64 `json:"theta"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.ComponentName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "component_name")
	}
	if cfg.Motion == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "motion")
	}
	if len(cfg.Poses) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "poses")
	}

	seen := make(map[string]bool, len(cfg.Poses))
	for i, p := range cfg.Poses {
		if p.Name == "" {
			return nil, nil, fmt.Errorf("%s: poses[%d] is missing required field \"name\"", path, i)
		}
		if seen[p.Name] {
			return nil, nil, fmt.Errorf("%s: poses[%d] has duplicate name %q", path, i, p.Name)
		}
		seen[p.Name] = true
	}

	deps := []string{cfg.ComponentName}
	if cfg.Motion == "builtin" {
		deps = append(deps, motion.Named("builtin").String())
	} else {
		deps = append(deps, cfg.Motion)
	}
	return deps, nil, nil
}

type multiPosesExecutionSwitch struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name      resource.Name
	logger    logging.Logger
	cfg       *Config
	motion    motion.Service
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

	motionSvc, err := motion.FromProvider(deps, conf.Motion)
	if err != nil {
		return nil, err
	}

	poseNames := make([]string, len(conf.Poses))
	for i, p := range conf.Poses {
		poseNames[i] = p.Name
	}

	return &multiPosesExecutionSwitch{
		name:      rawConf.ResourceName(),
		logger:    logger,
		cfg:       conf,
		motion:    motionSvc,
		poseNames: poseNames,
	}, nil
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
		return nil, fmt.Errorf("unknown pose name %q", name)
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
	if position >= uint32(len(s.poseNames)) {
		return fmt.Errorf("requested position %d is out of range (max %d)", position, len(s.poseNames)-1)
	}
	if !s.executing.CompareAndSwap(false, true) {
		return errors.New("switch is currently executing")
	}
	defer s.executing.Store(false)

	p := s.cfg.Poses[position]
	s.logger.Infof("moving %s to pose %q (index %d)", s.cfg.ComponentName, p.Name, position)

	pose := spatialmath.NewPose(
		r3.Vector{X: p.X, Y: p.Y, Z: p.Z},
		&spatialmath.OrientationVectorDegrees{OX: p.OX, OY: p.OY, OZ: p.OZ, Theta: p.Theta},
	)
	destination := referenceframe.NewPoseInFrame(s.cfg.ReferenceFrame, pose)

	if _, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.cfg.ComponentName,
		Destination:   destination,
	}); err != nil {
		return fmt.Errorf("failed to move to pose %q: %w", p.Name, err)
	}

	s.mu.Lock()
	s.position = position
	s.mu.Unlock()
	return nil
}
