include <out.scad>;

$fn = 90; // high resolution

LineHeight = 0.03*meter;
LineWidth = 0.03*meter;
FieldThickness = 0.02*meter;
Intersect = 0.001 * meter;
Noderadius = LineWidth * 0.5;

// bigger than usual, but print better than at actual scale
WheelRadius = 40;
WheelDepth = 20;

module robot(pos, focused=false, start=false) {
    // color("black")
    translate([pos[0], pos[1], 0]) {
        cylinder(r=RobotRadius, h=RobotHeight);
        for (i = [0:4]) {
            translate([RobotRadius,0,WheelRadius])
            rotate([0, 90, 0])
            color("green")
            cylinder(r=WheelRadius, h=WheelDepth);
        }

        // draw a target symbol on top of the destination robot
        if (focused || start) {
            translate([0,0,RobotHeight]) {
                f_height = LineHeight * 0.75;
                dr = 0.15;
                if (focused) {
                    for (r = [RobotRadius * dr*3, RobotRadius * dr*5, RobotRadius*dr]) {
                        difference() {
                            cylinder(r=r,h=f_height);
                            cylinder(r=r - dr*RobotRadius,h=f_height*2);
                        }
                    }
                } else if (start) {
                    cylinder(r=40, h=f_height);
                }
            }
        }
    }
}

// segment of an rrt tree
module line(from, to, sizeMultiplier=1) {
    w = LineWidth * sizeMultiplier;

    diff = [to[0] - from[0], to[1] - from[1]];
    dist = norm(diff);
    angle = atan2(diff[1], diff[0]);
    // translate to the start position
    translate([from[0], from[1], 0])
    rotate([0, 0, angle])
    // centers the line on the y-axis
    translate([0, -w/2,0])
    cube([dist, w, LineHeight]);
}

module node(pos) {
    translate([pos[0], pos[1], 0])
    cylinder(r=Noderadius, h=LineHeight);
}
// first print: 0.1 - way too small, can't see the rrt lines at all
// second print: close to right @ 0.03 scale
// Everything is drawn in units of mm at the size of an actual field - many meters.
// We scale it down significantly to a printable size
scaled_length = 110;
scale(scaled_length / FieldLength) {

    // field
    color("green")
    cube([FieldWidth, FieldLength, FieldThickness]);

    translate([0,0,FieldThickness - Intersect]) {
        // draw obstacle robots
        for (r = ObstacleRobots) {
            robot(r);
        }

        // start position
        robot(StartPos, start=true);

        // goal position
        robot(GoalPos, focused=true);

        // all lines in the rrt
        for (l = RRT_Lines) {
            line(l[0], l[1], 1);
        }

        // nodes
        for (n = RRT_Nodes) {
            node(n);
        }

        // draw lines in the solution thicker than the rest
        for (i = [0:len(RRT_Solution)-2]) {
            line(RRT_Solution[i], RRT_Solution[i+1], 1.5);
        }


        // translate([])
        s = 12.5;
        sx = 2;
        gw = 1*meter;
        gd = 0.415*meter;
        translate([FieldWidth / 2, FieldLength-gd, 0])
        scale([s*sx,s,s])
        import("goal.stl");

        wall_h = 30;
        cube([wall_h, FieldLength, wall_h]);

        translate([FieldWidth - wall_h, 0, 0])
        cube([wall_h, FieldLength, wall_h]);

        cube([FieldWidth, wall_h, wall_h]);

        translate([0, FieldLength - wall_h, 0])
        cube([FieldWidth, wall_h, wall_h]);
    }
}

