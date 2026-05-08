package cupmover

import (
	"context"
	"errors"
	"fmt"
	"sync/atomic"
	"time"

	"go.viam.com/rdk/components/gripper"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

var CupMover = resource.NewModel("viam-labs", "cup-mover", "cup-mover")

func init() {
	resource.RegisterService(generic.API, CupMover,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newCupMover,
		},
	)
}

type Config struct {
	resource.AlwaysRebuild
	PoseSwitchName string  `json:"pose_switch_name"`
	GripperName    string  `json:"gripper_name,omitempty"`
	Steps          []Step  `json:"steps,omitempty"`
	PauseSecs      float64 `json:"pause_secs,omitempty"`
}

// Step is one entry in the run sequence: move to Pose, then optionally do
// something with the gripper. Grip values: "grab", "open", "open_grab", or ""
// (no gripper action). Same pose may appear multiple times with different grips.
type Step struct {
	Pose string `json:"pose"`
	Grip string `json:"grip,omitempty"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.PoseSwitchName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "pose_switch_name")
	}
	deps := []string{cfg.PoseSwitchName}
	if cfg.GripperName != "" {
		deps = append(deps, cfg.GripperName)
	}
	return deps, nil, nil
}

type cupMover struct {
	resource.AlwaysRebuild

	name    resource.Name
	logger  logging.Logger
	cfg     *Config
	sw      toggleswitch.Switch
	gripper gripper.Gripper

	running    atomic.Bool
	cancelCtx  context.Context
	cancelFunc func()
}

func newCupMover(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}

	sw, err := toggleswitch.FromProvider(deps, conf.PoseSwitchName)
	if err != nil {
		return nil, fmt.Errorf("getting pose switch %q: %w", conf.PoseSwitchName, err)
	}

	var grip gripper.Gripper
	if conf.GripperName != "" {
		grip, err = gripper.FromProvider(deps, conf.GripperName)
		if err != nil {
			return nil, fmt.Errorf("getting gripper %q: %w", conf.GripperName, err)
		}
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	return &cupMover{
		name:       rawConf.ResourceName(),
		logger:     logger,
		cfg:        conf,
		sw:         sw,
		gripper:    grip,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}, nil
}

func (s *cupMover) Name() resource.Name {
	return s.name
}

func (s *cupMover) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["run"]; ok {
		visited, err := s.run(ctx)
		if err != nil {
			return nil, err
		}
		return map[string]interface{}{"visited": visited}, nil
	}
	return nil, fmt.Errorf("unknown command, supported: run")
}

// run walks through the configured steps in order. At the start of every run
// the gripper is opened once (if configured). Each step moves to its pose and
// then performs the step's gripper action (if any). Returns the list of pose
// names visited.
func (s *cupMover) run(ctx context.Context) ([]string, error) {
	if !s.running.CompareAndSwap(false, true) {
		return nil, errors.New("cup-mover is already running")
	}
	defer s.running.Store(false)

	_, names, err := s.sw.GetNumberOfPositions(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("getting pose names: %w", err)
	}
	if len(names) == 0 {
		return nil, errors.New("pose switch has no positions")
	}

	steps := s.cfg.Steps
	if len(steps) == 0 {
		steps = make([]Step, len(names))
		for i, n := range names {
			steps[i] = Step{Pose: n}
		}
	}

	indexByName := make(map[string]uint32, len(names))
	for i, n := range names {
		indexByName[n] = uint32(i)
	}

	if s.gripper != nil {
		if err := s.gripper.Open(ctx, nil); err != nil {
			return nil, fmt.Errorf("opening gripper at start of run: %w", err)
		}
	}

	pause := time.Duration(s.cfg.PauseSecs * float64(time.Second))
	visited := make([]string, 0, len(steps))

	for i, step := range steps {
		idx, ok := indexByName[step.Pose]
		if !ok {
			return visited, fmt.Errorf("pose %q not found on switch %q", step.Pose, s.cfg.PoseSwitchName)
		}

		s.logger.Infof("step %d/%d: %s (grip=%q)", i+1, len(steps), step.Pose, step.Grip)

		if err := s.sw.SetPosition(ctx, idx, nil); err != nil {
			return visited, fmt.Errorf("moving to %q: %w", step.Pose, err)
		}
		visited = append(visited, step.Pose)

		if err := s.applyGrip(ctx, step); err != nil {
			return visited, err
		}

		if pause > 0 && i < len(steps)-1 {
			select {
			case <-ctx.Done():
				return visited, ctx.Err()
			case <-s.cancelCtx.Done():
				return visited, s.cancelCtx.Err()
			case <-time.After(pause):
			}
		}
	}

	s.logger.Infof("done: visited %d pose(s)", len(visited))
	return visited, nil
}

func (s *cupMover) applyGrip(ctx context.Context, step Step) error {
	if step.Grip == "" {
		return nil
	}
	if s.gripper == nil {
		return fmt.Errorf("step %q requests grip=%q but no gripper is configured", step.Pose, step.Grip)
	}
	switch step.Grip {
	case "open":
		if err := s.gripper.Open(ctx, nil); err != nil {
			return fmt.Errorf("opening gripper at %q: %w", step.Pose, err)
		}
	case "grab":
		if _, err := s.gripper.Grab(ctx, nil); err != nil {
			return fmt.Errorf("grabbing at %q: %w", step.Pose, err)
		}
	case "open_grab":
		if err := s.gripper.Open(ctx, nil); err != nil {
			return fmt.Errorf("opening gripper at %q: %w", step.Pose, err)
		}

		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-s.cancelCtx.Done():
			return s.cancelCtx.Err()
		case <-time.After(time.Second):
		}

		if _, err := s.gripper.Grab(ctx, nil); err != nil {
			return fmt.Errorf("grabbing at %q: %w", step.Pose, err)
		}
	default:
		return fmt.Errorf("unknown grip %q at step %q (supported: open, grab, open_grab)", step.Grip, step.Pose)
	}
	return nil
}

func (s *cupMover) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{"running": s.running.Load()}, nil
}

func (s *cupMover) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
