package main

import (
	"encoding/json"
	"flag"
	"os"

	"github.com/JoshPattman/jcode"
	"github.com/gopxl/pixel"
	"github.com/gopxl/pixel/imdraw"
	"github.com/gopxl/pixel/pixelgl"
	"golang.org/x/image/colornames"
)

func main() {
	pixelgl.Run(run)
}

func run() {

	simSize := flag.Float64("dim", 16, "The number of units in size that the sim is wide and tall")
	winSize := flag.Float64("win", 800, "The width/height of the window")
	robotType := flag.String("robot", "picasso", "The robot to use, in a comma separated list")
	robotFile := flag.String("robot-lib", "robots.json", "The robot library file")
	paintThickness := flag.Float64("paint", 0.1, "The thickness (in units, NOT pixels) of the paint brush trail")

	flag.Parse()

	robots := make(RobotIndex)

	// Load the robots index
	if _, err := os.Stat(*robotFile); err != nil {
		robots = DefaultRobotIndex
		f, err := os.Create(*robotFile)
		if err != nil {
			panic(err)
		}
		defer f.Close()
		enc := json.NewEncoder(f)
		enc.SetIndent("", "\t")
		if err := enc.Encode(robots); err != nil {
			panic(err)
		}
	} else {
		f, err := os.Open(*robotFile)
		if err != nil {
			panic(err)
		}
		defer f.Close()
		if err := json.NewDecoder(f).Decode(&robots); err != nil {
			panic(err)
		}
	}

	// Start reading instructions from stdin
	instructions := jcode.BeginInstructionProcessing(os.Stdin, 10)
	instructionTelem := jcode.NewEncoder(os.Stdout)

	// Init window
	cfg := pixelgl.WindowConfig{
		Title:  "Picassim",
		Bounds: pixel.R(0, 0, *winSize, *winSize),
	}
	win, err := pixelgl.NewWindow(cfg)
	if err != nil {
		panic(err)
	}

	// Setup robot state
	tsm := &TargetStateMachine{}
	tsm.Reset(jcode.Waypoint{YPos: *simSize / 2})

	builder, ok := robots[*robotType]

	if !ok {
		panic("bad robot")
	}

	rbt := builder.BuildRobot(jcode.Waypoint{YPos: *simSize / 2})

	// Setup drawing
	imd := imdraw.New(nil)
	drawPaths := make([][]pixel.Vec, 0)
	lastPenMode := jcode.PenUp
	frameCounter := int64(0)

	// Loop
	for !win.Closed() {
		// Update window
		win.Update()
		win.Clear(colornames.Black)

		// Update target state
		if !tsm.Update(instructions, instructionTelem) {
			return
		}
		tspWpPos := tsm.TargetPosition()
		tspWpVel := tsm.TargetVelocity()
		tsmPixelPos := pixel.V(tspWpPos.XPos, tspWpPos.YPos)
		tsmPixelVel := pixel.V(tspWpVel.XPos, tspWpVel.YPos)
		rbt.Update(tsmPixelPos, tsmPixelVel)
		rbtPixelPos := rbt.EEPos()

		// Deal with drawing paths
		justPenUp := (tsm.PenMode == jcode.PenUp && lastPenMode == jcode.PenDown)
		if tsm.PenMode == jcode.PenDown || justPenUp {
			if lastPenMode == jcode.PenUp {
				// Need to create a new path
				drawPaths = append(drawPaths, []pixel.Vec{})
			}
			if frameCounter%60 == 0 || justPenUp {
				drawPaths[len(drawPaths)-1] = append(drawPaths[len(drawPaths)-1], rbtPixelPos)
			}
			frameCounter++
		} else {
			frameCounter = 0
		}
		lastPenMode = tsm.PenMode

		// Setup imdraw
		imd.Clear()
		imdSf := *winSize / *simSize
		imd.SetMatrix(pixel.IM.Scaled(pixel.ZV, imdSf).Moved(pixel.V(win.Bounds().Center().X, 0)))
		gridLineThickness := 1.0 / imdSf

		// Draw grid
		imd.Color = colornames.Gray
		for x := -*simSize; x <= *simSize; x++ {
			imd.Push(pixel.V(x, -100), pixel.V(x, 100))
			imd.Line(gridLineThickness)
		}
		for y := 0.0; y <= (*simSize)*2; y++ {
			imd.Push(pixel.V(-100, y), pixel.V(100, y))
			imd.Line(gridLineThickness)
		}

		// Draw strokes
		imd.Color = colornames.Wheat
		for _, stroke := range drawPaths {
			imd.Push(stroke...)
			imd.Line(*paintThickness)
		}

		// Draw origin
		imd.Color = colornames.Gray
		imd.Push(pixel.V(0, 0))
		imd.Circle(1, gridLineThickness)

		// Draw robot
		rbt.Draw(imd)

		// Draw target
		imd.Color = colornames.Green
		imd.Push(tsmPixelPos)
		imd.Circle(0.5, gridLineThickness)
		imd.Draw(win)
	}
}
