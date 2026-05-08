package multiposesexecutionswitch

import (
	"strings"
	"testing"
)

func TestValidate(t *testing.T) {
	pose := Pose{Name: "a", X: 100, Y: 200, Z: 300, OZ: 1, Theta: 45}

	tests := []struct {
		name    string
		cfg     Config
		wantErr string
	}{
		{
			name:    "missing component_name",
			cfg:     Config{Motion: "builtin", Poses: []Pose{pose}},
			wantErr: "component_name",
		},
		{
			name:    "missing motion",
			cfg:     Config{ComponentName: "arm", Poses: []Pose{pose}},
			wantErr: "motion",
		},
		{
			name:    "no poses",
			cfg:     Config{ComponentName: "arm", Motion: "builtin"},
			wantErr: "poses",
		},
		{
			name: "missing name",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Poses: []Pose{{X: 1}},
			},
			wantErr: `"name"`,
		},
		{
			name: "duplicate name",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Poses: []Pose{pose, pose},
			},
			wantErr: "duplicate",
		},
		{
			name: "valid single pose",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Poses: []Pose{pose},
			},
		},
		{
			name: "valid multiple poses",
			cfg: Config{
				ComponentName: "arm", Motion: "builtin",
				Poses: []Pose{
					{Name: "pickup", Z: 100, OZ: 1},
					{Name: "p1", X: 100, Z: 200, OZ: 1},
					{Name: "p2", X: 200, Z: 200, OZ: 1},
					{Name: "p3", X: 300, Z: 200, OZ: 1},
					{Name: "putdown", X: 300, Z: 100, OZ: 1},
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
