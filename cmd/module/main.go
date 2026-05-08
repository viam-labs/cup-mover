package main

import (
	cupmover "github.com/viam-labs/cup-mover"
	"github.com/viam-labs/cup-mover/dialcontrolmotion"
	"github.com/viam-labs/cup-mover/multiposesexecutionswitch"

	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/generic"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: generic.API, Model: cupmover.CupMover},
		resource.APIModel{API: toggleswitch.API, Model: multiposesexecutionswitch.Model},
		resource.APIModel{API: generic.API, Model: dialcontrolmotion.Model},
	)
}
