package multiposesexecutionswitch

import (
	"strings"
	"testing"
)

func TestValidate(t *testing.T) {
	pose := Waypoint{Name: "a", X: 100, Y: 200, Z: 300, OZ: 1, Theta: 45}

	tests := []struct {
		name    string
		cfg     Config
		wantErr string
	}{
		{
			name:    "missing component_name",
			cfg:     Config{Motion: "builtin", Waypoints: []Waypoint{pose}},
			wantErr: "component_name",
		},
		{
			name:    "missing motion in pose mode",
			cfg:     Config{ComponentName: "arm", Waypoints: []Waypoint{pose}},
			wantErr: "motion",
		},
		{
			name:    "no waypoints",
			cfg:     Config{ComponentName: "arm", Motion: "builtin"},
			wantErr: "poses",
		},
		{
			name: "missing name",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Waypoints: []Waypoint{{X: 1}},
			},
			wantErr: `"name"`,
		},
		{
			name: "duplicate name",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Waypoints: []Waypoint{pose, pose},
			},
			wantErr: "duplicate",
		},
		{
			name: "unknown mode",
			cfg: Config{
				ComponentName: "arm", Mode: "cartesian",
				Waypoints: []Waypoint{pose},
			},
			wantErr: "unknown mode",
		},
		{
			name: "joint mode missing joints",
			cfg: Config{
				ComponentName: "arm", Mode: ModeJoint,
				Waypoints: []Waypoint{{Name: "a"}},
			},
			wantErr: "joints",
		},
		{
			name: "valid single pose",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Waypoints: []Waypoint{pose},
			},
		},
		{
			name: "valid multiple poses",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Waypoints: []Waypoint{
					{Name: "pickup", Z: 100, OZ: 1},
					{Name: "p1", X: 100, Z: 200, OZ: 1},
					{Name: "p2", X: 200, Z: 200, OZ: 1},
					{Name: "p3", X: 300, Z: 200, OZ: 1},
					{Name: "putdown", X: 300, Z: 100, OZ: 1},
				},
			},
		},
		{
			name: "valid joint mode (no motion required)",
			cfg: Config{
				ComponentName: "arm", Mode: ModeJoint,
				Waypoints: []Waypoint{
					{Name: "home", Joints: []float64{0, 0, 0, 0, 0, 0}},
					{Name: "pickup", Joints: []float64{0, 1.2, -0.5, 0, 0.3, 0}},
				},
			},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			_, _, err := tt.cfg.Validate("test")
			if tt.wantErr == "" {
				if err != nil {
					t.Fatalf("unexpected error: %v", err)
				}
				return
			}
			if err == nil {
				t.Fatal("expected error, got nil")
			}
			if got := err.Error(); !strings.Contains(got, tt.wantErr) {
				t.Fatalf("error %q does not contain %q", got, tt.wantErr)
			}
		})
	}
}
